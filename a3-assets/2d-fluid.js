/**
 * Copyright 2022 Yuki Koyama
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * This code simulates incompressible fluid in a 2D squire domain using the Stable Fluids (semi-Lagrangian) approach [1]. In particular, this code mostly follows the follow-up document [2].
 *
 * - [1] Jos Stam. Stable Fluids. In Proceedings of SIGGRAPH 1999. https://www.dgp.toronto.edu/public_user/stam/reality/Research/pdf/ns.pdf
 * - [2] Jos Stam. Real-Time Fluid Dynamics for Games. In Proceedings of the Game Developer Conference 2003. https://www.dgp.toronto.edu/public_user/stam/reality/Research/pdf/GDC03.pdf
 */

 const canvas = document.getElementById("myCanvas");
 const context = canvas.getContext("2d");
 
 /**
  * 空間離散化の解像度
  */
 const numCells = 64;
 
 /**
  * セルの総数（ここでは便宜上境界にもセルを配置している)
  */
 const size = (numCells + 2) * (numCells + 2);
 
 /**
  * 速度（x方向）を格納をする配列
  */
 let u = new Float32Array(size);
 
 /**
  * 速度（y方向）を格納をする配列
  */
 let v = new Float32Array(size);
 
 /**
  * 可視化に用いる物質（例えば煙など）の密度（計算対象の流体そのものの密度とは異なる）を格納をする配列
  */
 let d = new Float32Array(size);
 
 let uPrev = new Float32Array(size);
 let vPrev = new Float32Array(size);
 let dPrev = new Float32Array(size);
 
 const dSource = new Float32Array(size);
 const uSource = new Float32Array(size);
 const vSource = new Float32Array(size);
 
 const numSubSteps = 8;
 const dt = 1.0 / (numSubSteps * 30.0);
 
 function clamp(x, x_min, x_max) {
   return Math.max(x_min, Math.min(x, x_max));
 }
 
 function index(i, j) {
   return i + (numCells + 2) * j;
 }
 
 /**
  * addSource - 対象となるデータに値を足す
  *
  * @param  {type} x 足される対象となる配列
  * @param  {type} s 足す値を格納した配列
  */
 function addSource(x, s) {
   for (let i = 0; i < size; ++i) {
     x[i] += dt * s[i];
   }
 }
 
 function isInBox(i, j) {
   let half = Math.floor(numCells / 2);
   if (i >= half - 1 && i <= half + 1) {
     if (
       j >= 2 * Math.floor(numCells / 5) &&
       j <= 3 * Math.floor(numCells / 5)
     ) {
       return true;
     }
   }
   return false;
 }
 
 /**
  * setBoundary - 境界条件を適用する
  *
  * @param  {type} x 境界条件を適用する対象となる配列
  * @param  {type} boundaryType 'continuous', 'horizontal_wall', 'vertical_wall' のいずれか
  */
 function setBoundary(x, boundaryType) {
   // 上下のエッジについて境界条件を適用する
   for (let i = 1; i <= numCells; ++i) {
     if (boundaryType == "vertical_wall") {
       x[index(i, 0)] = -x[index(i, 1)];
       x[index(i, numCells + 1)] = -x[index(i, numCells)];
     } else {
       x[index(i, 0)] = x[index(i, 1)];
       x[index(i, numCells + 1)] = x[index(i, numCells)];
     }
   }
   // 左右のエッジについて境界条件を適用する
   for (let j = 1; j <= numCells; ++j) {
     if (boundaryType == "horizontal_wall") {
       x[index(0, j)] = -x[index(1, j)];
       x[index(numCells + 1, j)] = -x[index(numCells, j)];
     } else {
       x[index(0, j)] = x[index(1, j)];
       x[index(numCells + 1, j)] = x[index(numCells, j)];
     }
   }
 
   // 四隅のセルには近傍のエッジのセルの平均値を代入しておく
   x[index(0, 0)] = 0.5 * (x[index(0, 1)] + x[index(1, 0)]);
   x[index(numCells + 1, 0)] =
     0.5 * (x[index(numCells + 1, 1)] + x[index(numCells, 0)]);
   x[index(0, numCells + 1)] =
     0.5 * (x[index(0, numCells)] + x[index(1, numCells + 1)]);
   x[index(numCells + 1, numCells + 1)] =
     0.5 * (x[index(numCells + 1, numCells)] + x[index(numCells, numCells + 1)]);
 
   // Add an obstacle to see Kármán's vortex
   for (
     let j = 2 * Math.floor(numCells / 5);
     j <= 3 * Math.floor(numCells / 5);
     ++j
   ) {
     let half = Math.floor(numCells / 2);
     if (boundaryType == "horizontal_wall") {
       x[index(half - 1, j)] = -x[index(half - 2, j)];
       x[index(half + 1, j)] = -x[index(half + 2, j)];
     } else {
       x[index(half - 1, j)] = x[index(half - 2, j)];
       x[index(half + 1, j)] = x[index(half + 2, j)];
     }
   }
 
   for (
     let i = Math.floor(numCells / 2) - 1;
     i <= Math.floor(numCells / 2) + 1;
     ++i
   ) {
     if (boundaryType == "horizontal_wall") {
       x[index(i, 2 * Math.floor(numCells / 5))] =
         -x[index(i, 2 * Math.floor(numCells / 5) - 1)];
       x[index(i, 3 * Math.floor(numCells / 5))] =
         -x[index(i, 3 * Math.floor(numCells / 5) + 1)];
     } else {
       x[index(i, 2 * Math.floor(numCells / 5))] =
         x[index(i, 2 * Math.floor(numCells / 5) - 1)];
       x[index(i, 3 * Math.floor(numCells / 5))] =
         x[index(i, 3 * Math.floor(numCells / 5) + 1)];
     }
   }
 }
 
 /**
  * diffuse - 拡散を計算する
  *
  * @param  {Float32Array} x 計算後のデータが格納される配列
  * @param  {Float32Array} x_0 計算前のデータが格納されている配列
  * @param  {string} boundaryType 'continuous', 'horizontal_wall', 'vertical_wall' のいずれか
  */
 function diffuse(x, x_0, boundaryType) {
   const diffusion_rate = 0.0001;
   const a = dt * diffusion_rate * numCells * numCells;
 
   // ガウスザイデル法における反復回数
   const numIters = 4;
 
   // ガウスザイデル法を使って拡散方程式を解く
   for (let k = 0; k < numIters; ++k) {
     // 各セルにガウスザイデル法の更新式を適用する
     for (let i = 1; i <= numCells; ++i) {
       for (let j = 1; j <= numCells; ++j) {
         if (isInBox(i, j)) continue;
         x[index(i, j)] =
           x_0[index(i, j)] +
           a *
             (x[index(i - 1, j)] +
               x[index(i + 1, j)] +
               x[index(i, j - 1)] +
               x[index(i, j + 1)]);
         x[index(i, j)] /= 1.0 + 4.0 * a;
       }
     }
 
     // 境界条件を設定する
     setBoundary(x, boundaryType);
   }
 }
 
 /**
  * advect - 移流を計算する
  *
  * この実装では、Semi-Lagrangianなバックトレースを行い、周囲のセルから値の補間を行う。
  *
  * バックトレースには単純な前進オイラー法を用いる。なお、精度の観点でバックトレースにはルンゲクッタ法 [1] (RK2など) を使う方が好ましいとされている。
  *
  * 補間にはバイリニア補間 [2] を用いる。Monotonic Cubic Interpolation [3] などの手法を用いる方がより好ましい。
  *
  * - [1] https://en.wikipedia.org/wiki/Bilinear_interpolation
  * - [2] https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
  * - [3] Ronald Fedkiw, Jos Stam, and Henrik Wann Jensen. 2001. Visual Simulation of Smoke. In Proc. SIGGRAPH 2001. https://dl.acm.org/doi/10.1145/383259.383260
  *
  * @param  {Float32Array} x 計算後のデータが格納される配列
  * @param  {Float32Array} x_0 計算前のデータが格納されている配列
  * @param  {Float32Array} u_0 速度のx成分が格納されている配列
  * @param  {Float32Array} v_0 速度のy成分が格納されている配列
  * @param  {string} boundaryType 'continuous', 'horizontal_wall', 'vertical_wall' のいずれか
  */
 function advect(x, x_0, u_0, v_0, boundaryType) {
   let dt0 = dt * numCells;
   for (let i = 1; i <= numCells; ++i) {
     for (let j = 1; j <= numCells; ++j) {
       if (isInBox(i, j)) continue;
       // バックトレースした先の座標値 (p_x, p_y) を計算する
       let p_x = i - dt0 * u_0[index(i, j)];
       let p_y = j - dt0 * v_0[index(i, j)];
       // コーナーケース（境界付近）の処理をする（計算領域からはみ出ていた場合）
       p_x = clamp(p_x, 0.5, numCells + 0.5);
       p_y = clamp(p_y, 0.5, numCells + 0.5);
       // バイリニア補間の対象となるセルのインデックスを計算する
       let x_1 = parseInt(p_x);
       let x_2 = x_1 + 1;
       let y_1 = parseInt(p_y);
       let y_2 = y_1 + 1;
       // バイリニア補間のウェイトを計算する
       let w_11 = (x_2 - p_x) * (y_2 - p_y);
       let w_21 = (p_x - x_1) * (y_2 - p_y);
       let w_12 = (x_2 - p_x) * (p_y - y_1);
       let w_22 = (p_x - x_1) * (p_y - y_1);
       // バイニリア補間を計算する
       x[index(i, j)] =
         w_11 * x_0[index(x_1, y_1)] +
         w_12 * x_0[index(x_1, y_2)] +
         w_21 * x_0[index(x_2, y_1)] +
         w_22 * x_0[index(x_2, y_2)];
     }
   }
 
   setBoundary(x, boundaryType);
 }
 
 /**
  * project - 速度場の発散をゼロにする
  *
  * @param  {Float32Array} u 速度のx成分が格納されている配列
  * @param  {Float32Array} v 速度のy成分が格納されている配列
  * @param  {Float32Array} p 途中計算結果を格納するための確保済みの適当な配列
  * @param  {Float32Array} div 途中計算結果を格納するための確保済みの適当な配列
  */
 function project(u, v, p, div) {
   // ガウスザイデル法における反復回数
   const numIters = 10;
 
   // 解を格納するためのバッファをゼロで初期化しておく
   for (let i = 0; i < size; ++i) {
     p[i] = 0.0;
   }
 
   // 各セルの発散を中心差分法により計算する
   let h = 1.0 / numCells;
   for (let i = 1; i <= numCells; ++i) {
     for (let j = 1; j <= numCells; ++j) {
       if (isInBox(i, j)) continue;
       div[index(i, j)] =
         -0.5 *
         h *
         (u[index(i + 1, j)] -
           u[index(i - 1, j)] +
           v[index(i, j + 1)] -
           v[index(i, j - 1)]);
     }
   }
   setBoundary(div, "continuous");
 
   // ガウスザイデル法によってポアソン方程式を解くことでスカラー場を計算する
   for (let k = 0; k < numIters; ++k) {
     // 各セルにガウスザイデル法の更新式を適用する
     for (let i = 1; i <= numCells; ++i) {
       for (let j = 1; j <= numCells; ++j) {
         if (isInBox(i, j)) continue;
         p[index(i, j)] =
           (div[index(i, j)] +
             p[index(i - 1, j)] +
             p[index(i + 1, j)] +
             p[index(i, j - 1)] +
             p[index(i, j + 1)]) /
           4;
       }
     }
 
     // 境界条件を設定する
     setBoundary(p, "continuous");
   }
 
   // 速度場から得られたスカラー場の勾配（中央差分法で計算）を引く
   for (let i = 1; i <= numCells; ++i) {
     for (let j = 1; j <= numCells; ++j) {
       if (isInBox(i, j)) continue;
       u[index(i, j)] -= (0.5 * (p[index(i + 1, j)] - p[index(i - 1, j)])) / h;
       v[index(i, j)] -= (0.5 * (p[index(i, j + 1)] - p[index(i, j - 1)])) / h;
     }
   }
   setBoundary(u, "horizontal_wall");
   setBoundary(u, "vertical_wall");
 }
 
 function densityStep() {
   addSource(d, dSource);
 
   [d, dPrev] = [dPrev, d];
   diffuse(d, dPrev, "continuous");
 
   [d, dPrev] = [dPrev, d];
   advect(d, dPrev, u, v, "continuous");
 }
 
 /**
  * velocityStep - 速度場を更新する
  *
  * ここでの実装は Stable Fluids (SIGGRAPH 1999) ではなく Real-Time Fluid Dynamics for Games (GDC 2003) に基づいている。後者の方が多少計算コストが上がるものの、より視覚的に優れた結果が得られるようである。
  */
 function velocityStep() {
   addSource(u, uSource);
   addSource(v, vSource);
 
   [u, uPrev] = [uPrev, u];
   [v, vPrev] = [vPrev, v];
   diffuse(u, uPrev, "horizontal_wall");
   diffuse(v, vPrev, "vertical_wall");
 
   project(u, v, uPrev, vPrev);
 
   [u, uPrev] = [uPrev, u];
   [v, vPrev] = [vPrev, v];
   advect(u, uPrev, uPrev, vPrev, "horizontal_wall");
   advect(v, vPrev, uPrev, vPrev, "vertical_wall");
 
   project(u, v, uPrev, vPrev);
 }
 
 function step() {
   velocityStep();
   densityStep();
 }
 
 function draw() {
   context.clearRect(0, 0, canvas.width, canvas.height);
 
   const cellWidth = canvas.width / (numCells + 2);
   const cellHeight = canvas.height / (numCells + 2);
 
   context.strokeStyle = "rgba(100, 100, 100, 0.2)";
   context.lineWidth = 2.0;
   for (let i = 1; i <= numCells; ++i) {
     for (let j = 1; j <= numCells; ++j) {
       // Draw a grid with its color-coded density
       const scale = 0.08;
       const rgb = evaluate_cmap(
         isInBox(i, j) ? 0.0 : clamp(d[index(i, j)] * scale, 0.0, 1.0),
         "viridis",
         false
       );
       const color = "rgb(" + rgb[0] + ", " + rgb[1] + ", " + rgb[2] + ", 1.0)";
 
       context.fillStyle = color;
       context.beginPath();
       context.rect(i * cellWidth, j * cellHeight, cellWidth, cellHeight);
       context.fill();
       context.stroke();
     }
   }
 
   // Visualize velocities
   const velColor = "rgba(255, 255, 255, 0.4)";
   context.strokeStyle = velColor;
   context.lineWidth = 2.0;
   for (let i = 1; i <= numCells; ++i) {
     for (let j = 1; j <= numCells; ++j) {
       const scale = 80.0;
 
       const centerX = (i + 0.5) * cellWidth;
       const centerY = (j + 0.5) * cellHeight;
       context.beginPath();
       context.moveTo(centerX, centerY);
       context.lineTo(
         centerX + u[index(i, j)] * scale,
         centerY + v[index(i, j)] * scale
       );
       context.stroke();
     }
   }
 }
 
 // Define variables for managing animation
 let frameCount = 0;
 let simTime = 0.0;
 let drawTime = 0.0;
 
 function update() {
   // 速度や密度のソースを指定する
   dSource.fill(0.0);
   uSource.fill(0.0);
   vSource.fill(0.0);
   dSource[index(numCells / 8, numCells / 2)] = 4000.0;
   uSource[index(numCells / 8, numCells / 2)] = 500.0;
 
   // シミュレーションを一ステップ前に進める
   const time_0 = performance.now();
   for (let i = 0; i < numSubSteps; ++i) {
     step();
   }
   const time_1 = performance.now();
 
   // 現在の状態を描画する
   draw();
   const time_2 = performance.now();
 
   // 計算にかかった時間を記録する
   simTime += time_1 - time_0;
   drawTime += time_2 - time_1;
 
   // 計算にかかった時間を表示する
   const frequency = 20;
   if (frameCount % frequency == 0) {
     document.getElementById("simTime").innerHTML = (
       simTime / frequency
     ).toFixed(3);
     document.getElementById("drawTime").innerHTML = (
       drawTime / frequency
     ).toFixed(3);
 
     simTime = 0.0;
     drawTime = 0.0;
   }
 
   // 描画結果を画像ファイルに保存する
   const exportImage = false;
   if (exportImage) {
     const safeWaitingTime = 200;
     const fileName = String(frameCount).padStart(4, "0") + ".png";
     const maxFrameCount = 180;
     const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
 
     canvas.toBlob(function (blob) {
       saveAs(blob, fileName);
     });
 
     if (++frameCount < maxFrameCount) {
       sleep(safeWaitingTime).then(() => {
         window.requestAnimationFrame(update);
       });
     }
     return;
   }
 
   // 再帰的に関数を呼び出すことでアニメーションを継続する
   const maxFrameCount = 1800;
   if (++frameCount < maxFrameCount) {
     window.requestAnimationFrame(update);
   }
 }
 
 /**
  * start - アニメーションを開始する
  */
 function start() {
   update();
 }
 