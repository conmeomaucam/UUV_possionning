# BÁO CÁO THIẾT KẾ KỸ THUẬT HỆ THỐNG ES-EKF TÍCH HỢP INS/DVL
**Ứng dụng Lý thuyết Joan Solà ước lượng đồng thời Vị trí, Vận tốc, Thái độ, Bias IMU, Hệ số tỷ lệ k và Góc Boresight DVL**
*(Tài liệu tham chiếu chuẩn: Joan Solà, "Quaternion kinematics for the error-state Kalman filter", 2017)*

---

## 1. TỔNG QUAN VÀ NGUYÊN LÝ CỐT LÕI CỦA BỘ LỌC ES-EKF

### 1.1. Tại sao lựa chọn Error-State Kalman Filter (ES-EKF)?
Theo phân tích của Joan Solà (Mục 5.1 & 5.2):
* **Trạng thái danh định (Nominal State - x):** Chứa các đại lượng động học tín hiệu lớn (vị trí p, vận tốc v, quaternion q). Khối này được tích phân phi tuyến trực tiếp từ dữ liệu IMU ở tần số cao (400 Hz).
* **Trạng thái sai số (Error State - dx):** Chứa các sai số tín hiệu nhỏ (delta_p, delta_v, delta_theta, delta_bias, delta_k, delta_phi_bs). Sai số luôn dao động quanh 0, đảm bảo tính tuyến tính hóa luôn chuẩn xác, không bao giờ bị hiện tượng kỳ dị (singularity) hay khóa trục (Gimbal lock).
* **Khử trôi liên tục (Closed-loop Feedback & Reset):** Ngay khi có số đo DVL ở chu kỳ chậm (5 Hz), bộ lọc ước lượng vector sai số `dx`, hồi tiếp ngay lập tức để bù trừ vào trạng thái danh định, sau đó **reset dx về 0** và cập nhật ma trận hiệp phương sai P.

### 1.2. Sơ đồ khối nguyên lý hệ thống (Phong cách Simulink)

> [!TIP]
> * **Để xem đồ họa màu tương tác trong VS Code:** Nhấn tổ hợp phím **`Ctrl + Shift + V`** (hoặc bấm biểu tượng **Open Preview** ở góc trên cùng bên phải màn hình).
> * **Xem trực tiếp ngay trong trình soạn thảo code:** Dưới đây là bản vẽ dạng khối hộp Simulink hoàn chỉnh:

```text
========================================================================================================================
                                 SƠ ĐỒ KHỐI NGUYÊN LÝ ES-EKF (DẠNG HỘP SIMULINK TRỰC QUAN)
========================================================================================================================

  [CẢM BIẾN IMU 400Hz]                                       (VÒNG TÍCH PHÂN INS DANH ĐỊNH - 400 Hz)
  +--------------------+   a_m, w_m     +-------+    a_sạch    +-----------------------+    v_NED, q
  | Gia tốc:  a_m      |--------------->| ( - ) |------------->|  BỘ TÍCH PHÂN INS     |----------------+---> [ĐẦU RA]
  | Con quay: omega_m  |                +-------+    w_sạch    |     (Khối 1/s)        |                |  Vị trí: p
  +--------------------+                    ^                  |  Tích phân Quaternion |                |  Vận tốc: v
                                            | a_b, omega_b     |  Tích phân v_NED, p   |                |  Thái độ: q
                                            | (Trừ Bias)       +-----------------------+                |
                                  +-------------------+                     ^                           |
                                  | BỘ TÍCH LŨY       |                     | (HỒI TIẾP 1: Sửa tức thời)|
                                  | BIAS IMU          |<-------+            | Sửa v_NED, q không cho trôi
                                  | (a_b, omega_b)    |        |            |                           |
                                  +-------------------+        | da_b, dw_b | dv, dq                    |
                                                               | (HỒI TIẾP 2)                           |
                                                               |            |                           |
  =============================================================|============|===========================|===============
  (CHU KỲ CHẬM 5 Hz - CẬP NHẬT ĐO LƯỜNG DVL)                   |            |                           |
                                                               |            |                           |
  [CẢM BIẾN DVL 5Hz]                                           |            |                           |
  +--------------------+   z_dvl (+)                           |            |                           |
  | Vận tốc đáy DVL    |------------+                          |            |                           v
  +--------------------+            |                          |            |                 +-------------------+
                                    v                          |            |                 |  MÔ HÌNH DVL h(x) |
                               +---------+   Phần dư res       |            |                 | v_dvl_pred =      |
                               |  ( - )  |--------------------+|            |                 | (1+k)*C_bs*Cnb*v  |
                               +---------+                     |            |                 +-------------------+
                                    ^                          v            |                           ^
                                    | v_pred (-)     +-------------------+  |                           |
                                    |                |  KALMAN GAIN K    |--+                           | k, phi_bs
                                    +----------------|   Kích thước      |                              |
                                                     |    (19 x 3)       |-----> [RESET dx -> 0]        |
                                                     +-------------------+                              |
                                                               |                                        |
                                                               | dk, dphi_bs                            |
                                                               | (HỒI TIẾP 3: Tích lũy tham số)         |
                                                               v                                        |
                                                     +-------------------+                              |
                                                     | BỘ TÍCH LŨY       |                              |
                                                     | THAM SỐ DVL       |------------------------------+
                                                     | k, phi_bs         |
                                                     +-------------------+
========================================================================================================================
```

---


#### Giải thích cơ chế luồng tín hiệu kiểu Simulink:
1. **Đường thuận (Feedforward - 400 Hz):**
   * Dữ liệu thô từ IMU đi qua bộ trừ bias `(-) a_b, omega_b`, sau đó đưa vào bộ tích phân quán tính `1/s` để liên tục tính ra vận tốc `v_NED` và thái độ `q`.
2. **Điểm so sánh (Summing Junction - 5 Hz):**
   * Vận tốc INS được xoay về khung thân và nhân với `(1 + k)` và ma trận góc lệch `phi_bs` để tạo ra vận tốc DVL dự báo `v_dvl_pred`.
   * Bộ trừ so sánh `z_dvl - v_dvl_pred` tạo ra tín hiệu **Phần dư (Residual)**.
3. **Bộ điều khiển lỗi (Kalman Gain K):**
   * Khuếch đại phần dư thành vector hiệu chỉnh `dx` gồm 19 thành phần.
4. **Ba nhánh hồi tiếp khép kín (Feedback Loops):**
   * **Nhánh 1 (Khử trôi INS):** Bơm `dv` và `dtheta` ngược về bộ tích phân INS để sửa đổi vận tốc tức thời $\rightarrow$ INS **không bao giờ bị trôi**.
   * **Nhánh 2 (Bù Bias IMU):** Bơm `da_b, domega_b` vào bộ tích lũy bias $\rightarrow$ Triệt tiêu sai số phần cứng IMU.
   * **Nhánh 3 (Tích lũy Tham số):** Bơm `dk` và `dphi_bs` vào bộ tích lũy tham số DVL $\rightarrow$ Kéo `k` và `Boresight` hội tụ dần từ 0 về đúng giá trị thật.


---

## 2. THIẾT KẾ CÁC VECTOR TRẠNG THÁI MỞ RỘNG

### 2.1. Vector Trạng thái Danh định (Nominal State Vector - 20 phần tử)
Bao gồm các đại lượng vật lý thực tế được lưu trữ:
```
x = [ p (3x1);        % Vị trí khung NED [Bắc, Đông, Xuống] (mét)
      v (3x1);        % Vận tốc khung NED [v_N, v_E, v_D] (m/s)
      q (4x1);        % Quaternion thái độ thân -> NED [q_w, q_x, q_y, q_z]
      a_b (3x1);      % Bias gia tốc kế khung thân [b_ax, b_ay, b_az] (m/s^2)
      omega_b (3x1);  % Bias con quay khung thân [b_gx, b_gy, b_gz] (rad/s)
      k (1x1);        % Hệ số tỷ lệ tốc độ DVL (không thứ nguyên)
      phi_bs (3x1) ]  % Góc lệch Boresight DVL [alpha_roll, beta_pitch, gamma_yaw] (rad)
```

### 2.2. Vector Trạng thái Sai số Mở rộng (Error State Vector - 19 phần tử)
Đặc trưng cho độ lệch của hệ thống:
```
dx = [ delta_p (3x1);        % Sai số vị trí (mét)
       delta_v (3x1);        % Sai số vận tốc (m/s)
       delta_theta (3x1);    % Sai số góc định hướng (rad)
       delta_ab (3x1);       % Sai số bias gia tốc (m/s^2)
       delta_wb (3x1);       % Sai số bias con quay (rad/s)
       delta_k (1x1);        % Sai số hệ số tỷ lệ k
       delta_phi_bs (3x1) ]  % Sai số góc Boresight [d_alpha, d_beta, d_gamma] (rad)
```
*(Ghi chú: Nếu kênh Roll alpha hoàn toàn không quan sát được trên quỹ đạo phẳng, vector có thể rút gọn thành 18 trạng thái bằng cách chỉ giữ lại beta và gamma).*

---

## 3. MÔ HÌNH TOÁN HỌC MA TRẬN DỰ BÁO (PREDICTION STEP - 400 HZ)

### 3.1. Phương trình vi phân trạng thái sai số liên tục
Mở rộng phương trình (238) trong tài liệu Joan Solà cho thêm k và Boresight:
```
d(delta_p) / dt       = delta_v
d(delta_v) / dt       = - R * [a_m - a_b]x * delta_theta - R * delta_ab - R * a_noise
d(delta_theta) / dt   = - [omega_m - omega_b]x * delta_theta - delta_wb - omega_noise
d(delta_ab) / dt      = w_ab  (Nhiễu ngẫu nhiên trôi bias gia tốc)
d(delta_wb) / dt      = w_wb  (Nhiễu ngẫu nhiên trôi bias con quay)
d(delta_k) / dt       = 0     (Tham số cơ khí cố định / biến thiên cực chậm)
d(delta_phi_bs) / dt  = 0     (Góc gá lắp cơ khí cố định)
```
Trong đó:
* `R = R{q}` là ma trận quay chuyển tọa độ từ thân sang NED (Direction Cosine Matrix).
* `[v]x` là ma trận phản đối xứng (skew-symmetric matrix) của vector v:
```
         [   0   -v(3)   v(2) ]
  [v]x = [  v(3)    0   -v(1) ]
         [ -v(2)  v(1)    0   ]
```

### 3.2. Cấu trúc Ma trận Chuyển trạng thái Rời rạc F_x (Kích thước 19x19)

Dưới đây là ma trận F_x rời rạc theo xấp xỉ bậc 1 (Euler-Taylor) của Solà (công thức 270 & 401):

```text
       delta_p   delta_v       delta_theta       delta_ab   delta_wb    delta_k   delta_phi_bs
     +---------+---------+---------------------+----------+----------+---------+--------------+
dp   |  I_3x3  |  I*dt   |        0_3x3        |  0_3x3   |  0_3x3   |  0_3x1  |    0_3x3     |
     +---------+---------+---------------------+----------+----------+---------+--------------+
dv   |  0_3x3  |  I_3x3  | -R*[a_m - a_b]x * dt|  -R*dt   |  0_3x3   |  0_3x1  |    0_3x3     |
     +---------+---------+---------------------+----------+----------+---------+--------------+
dth  |  0_3x3  |  0_3x3  | R{(w_m-w_b)*dt}^T   |  0_3x3   |  -I*dt   |  0_3x1  |    0_3x3     |
     +---------+---------+---------------------+----------+----------+---------+--------------+
dab  |  0_3x3  |  0_3x3  |        0_3x3        |  I_3x3   |  0_3x3   |  0_3x1  |    0_3x3     |
     +---------+---------+---------------------+----------+----------+---------+--------------+
dwb  |  0_3x3  |  0_3x3  |        0_3x3        |  0_3x3   |  I_3x3   |  0_3x1  |    0_3x3     |
     +---------+---------+---------------------+----------+----------+---------+--------------+
dk   |  0_1x3  |  0_1x3  |        0_1x3        |  0_1x3   |  0_1x3   |    1    |    0_1x3     |
     +---------+---------+---------------------+----------+----------+---------+--------------+
dbs  |  0_3x3  |  0_3x3  |        0_3x3        |  0_3x3   |  0_3x3   |  0_3x1  |    I_3x3     |
     +---------+---------+---------------------+----------+----------+---------+--------------+
```

### 3.3. Ma trận Nhiễu Quá trình Q (Process Noise Matrix)
Theo mục 5.4.2 và Phụ lục E trong tài liệu của Solà:
Các xung nhiễu ngẫu nhiên tác động vào vector sai số gồm 4 nguồn nhiễu:
1. `V_i = (acc_vrw)^2 * dt * I_3x3` (Nhiễu vận tốc góc từ VRW của gia tốc kế, m^2/s^2).
2. `Theta_i = (gyro_arw)^2 * dt * I_3x3` (Nhiễu góc từ ARW của con quay, rad^2).
3. `A_i = (acc_bias_walk)^2 * dt * I_3x3` (Nhiễu trôi bias gia tốc, m^2/s^4).
4. `Omega_i = (gyro_bias_walk)^2 * dt * I_3x3` (Nhiễu trôi bias con quay, rad^2/s^2).
5. `K_i = (sigma_k_walk)^2 * dt` (Mô hình random walk cực nhỏ cho k để thích nghi nhiệt độ, vd: 1e-10).
6. `BS_i = (sigma_bs_walk)^2 * dt * I_3x3` (Mô hình random walk cực nhỏ cho Boresight, vd: 1e-12).

Ma trận phân tán nhiễu `F_i` (19x13) và `Q_i` (13x13):
```text
           V_i      Theta_i    A_i     Omega_i    K_i    BS_i
        +---------+---------+-------+---------+-------+-------+
dp      |  0_3x3  |  0_3x3  | 0_3x3 |  0_3x3  | 0_3x1 | 0_3x3 |
dv      |  I_3x3  |  0_3x3  | 0_3x3 |  0_3x3  | 0_3x1 | 0_3x3 |
dtheta  |  0_3x3  |  I_3x3  | 0_3x3 |  0_3x3  | 0_3x1 | 0_3x3 |
dab     |  0_3x3  |  0_3x3  | I_3x3 |  0_3x3  | 0_3x1 | 0_3x3 |
domega  |  0_3x3  |  0_3x3  | 0_3x3 |  I_3x3  | 0_3x1 | 0_3x3 |
dk      |  0_1x3  |  0_1x3  | 0_1x3 |  0_1x3  |   1   | 0_1x3 |
dphi_bs |  0_3x3  |  0_3x3  | 0_3x3 |  0_3x3  | 0_3x1 | I_3x3 |
        +---------+---------+-------+---------+-------+-------+
```
Công thức cập nhật hiệp phương sai: `P = F_x * P * F_x^T + F_i * Q_i * F_i^T`.

---

## 4. MÔ HÌNH TOÁN HỌC MA TRẬN ĐO LƯỜNG DVL (UPDATE STEP - 5 HZ)

### 4.1. Hàm đo phi tuyến h(x) CÓ CÁNH TAY ĐÒN LEVER ARM l
Vận tốc dự báo mà đầu đo DVL sẽ nhìn thấy ở khung đo DVL (khi DVL cách IMU một vector cánh tay đòn `l = [lx, ly, lz]^T`):
```
h(x) = (1 + s) * C_b_dvl * [ R^T * v + omega_nb^b x l ]
```
Trong đó:
* Đổi tên hệ số tỷ lệ thành `s` (Scale Factor) để không nhầm với ma trận Kalman Gain `K`.
* `R^T * v` là vận tốc INS chiếu về khung thân IMU.
* `omega_nb^b = omega_m - omega_b` là vận tốc góc quay của tàu ở khung thân.
* `omega_nb^b x l` là **vận tốc quay do cánh tay đòn sinh ra** tại vị trí đầu đo DVL. Nếu thiếu số hạng này, khi tàu quay vòng (như lượn hình số 8 với `omega_z ≈ 8 độ/s, lx = 0.5m`), vận tốc trượt `0.07 m/s` sẽ bị đổ nhầm thành lỗi Boresight ảo tới `2.7 độ`!
* `v_L_b = R^T * v + omega_nb^b x l` là vận tốc tổng hợp tại tâm DVL ở khung thân.
* `C_b_dvl = I_3x3 - [phi_bs]x` là ma trận góc lệch Boresight chuyển từ thân IMU sang đầu đo DVL.

### 4.2. Khai triển Ma trận Đo Jacobian H Đầy Đủ (Kích thước 3x19 hoặc 3x22)
Ma trận Jacobian `H = dh / d(dx)` xác định độ nhạy của phép đo DVL đối với từng biến sai số.
Sử dụng quy tắc đạo hàm chuỗi của Joan Solà (Mục 6.1) và tính đến cả số hạng cánh tay đòn `omega x l`:

```text
H = [ H_p | H_v | H_theta | H_ab | H_wb | H_s | H_bs ]
```

Chi tiết từng khối con:
1. **Theo sai số vị trí (delta_p):**
   `H_p = 0_3x3` (Vận tốc DVL không phụ thuộc vị trí).
2. **Theo sai số vận tốc (delta_v):**
   `H_v = (1 + s) * C_b_dvl * R^T` (Kích thước 3x3).
3. **Theo sai số góc thái độ (delta_theta):**
   Do góc sai số cục bộ làm biến đổi ma trận quay: `R_true^T * v ≈ R^T * v + [R^T * v]x * delta_theta`.
   `H_theta = (1 + s) * C_b_dvl * [R^T * v]x` (Kích thước 3x3).
4. **Theo sai số Bias gia tốc (delta_ab):**
   `H_ab = 0_3x3`.
5. **Theo sai số Bias con quay (delta_wb) - ĐẠO HÀM QUA LEVER ARM:**
   Do `omega_nb_true = omega_m - omega_b - delta_wb`, số hạng `omega x l = - [l]x * omega` dẫn đến:
   `H_wb = (1 + s) * C_b_dvl * [l]x` (Kích thước 3x3 - ĐÂY LÀ KHỐI QUAN TRỌNG KHI CÓ LEVER ARM).
6. **Theo sai số Hệ số tỷ lệ s (delta_s):**
   Đạo hàm trực tiếp theo s:
   `H_s = C_b_dvl * v_L_b` (Kích thước 3x1).
7. **Theo sai số Góc Boresight (delta_phi_bs):**
   Do `C_b_dvl_true * v_L_b ≈ C_b_dvl * v_L_b + [v_L_b]x * delta_phi_bs`.
   `H_bs = (1 + s) * [v_L_b]x` (Kích thước 3x3).
8. **Tùy chọn mở rộng 22 trạng thái (nếu ước lượng cả Lever Arm l):**
   `H_l = -(1 + s) * C_b_dvl * [omega_nb^b]x` (Kích thước 3x3).

*Khuyến nghị:* Kiểm chứng ma trận giải tích `H` bằng đạo hàm số (Finite Difference) trong MATLAB để đảm bảo sai số `norm(H_analytic - H_numeric) < 1e-6`.

#### Biểu diễn toàn cảnh Ma trận H bằng mã ASCII:

```text
      delta_p   delta_v              delta_theta          dab        delta_wb             delta_s           delta_phi_bs
     +-------+--------------------+---------------------+-----+--------------------+------------------+---------------------+
z_x  | 0 0 0 | [                  | [                   | 000 | [                  | (C_b_dvl*v_Lb)_x | [                   |
z_y  | 0 0 0 | (1+s)*C_b_dvl*R^T  | (1+s)*C_bs*[R^Tv]x  | 000 | (1+s)*C_b_dvl*[l]x | (C_b_dvl*v_Lb)_y |   (1+s)*[v_Lb]x     |
z_z  | 0 0 0 |                    |                     | 000 |                    | (C_b_dvl*v_Lb)_z |                     |
     +-------+--------------------+---------------------+-----+--------------------+------------------+---------------------+
Col:   1:3           4:6                    7:9          10:12        13:15                 16                 17:19
```

### 4.3. Ma trận Nhiễu đo lường R_dvl (Kích thước 3x3)
Từ datasheet DVL Water Linked A125 (độ chính xác tiêu chuẩn 0.1% vận tốc + sàn nhiễu 0.1 mm/s):
```
sigma_d = 0.001 * norm(v_L_b) + 0.1e-3 (m/s)
R_dvl   = diag([sigma_d^2, sigma_d^2, sigma_d^2])
```

---

## 5. THIẾT KẾ BƯỚC HỒI TIẾP VÀ RESET THEO JOAN SOLÀ (MỤC 6.2 & 6.3)

Sau khi tính `dx = K * (z_dvl - h(x))` (với `dx` gồm 19 phần tử):

### 5.1. Bơm sai số vào trạng thái danh định (Injection) ĐẦY ĐỦ
```
p_NED   <- p_NED + dx(1:3)        % Sửa vị trí dp
v_NED   <- v_NED + dx(4:6)        % Sửa vận tốc dv
q       <- q (x) Exp( dx(7:9) )   % Phép nhân Quaternion Hamilton với vector quay sai số
a_b     <- a_b + dx(10:12)        % Bù Bias gia tốc
omega_b <- omega_b + dx(13:15)    % Bù Bias con quay
s       <- s + dx(16)             % TÍCH LŨY HỆ SỐ TỶ LỆ DVL s
phi_bs  <- phi_bs + dx(17:19)     % TÍCH LŨY GÓC LỆCH BORESIGHT
```
Trong đó hàm `Exp(dtheta)` cho góc nhỏ xấp xỉ bằng:
`dq = [ 1;  0.5 * dtheta ] / norm([ 1;  0.5 * dtheta ])`

### 5.2. Reset Error State và Cập nhật Covariance
Sau khi đã bơm sai số vào danh định:
1. **Reset vector sai số:** `dx <- zeros(19, 1)`.
2. **Cập nhật hiệp phương sai Joseph form:**
   ```
   P_joseph = (I - K*H) * P * (I - K*H)^T + K * R_dvl * K^T
   ```
3. **Biến đổi ma trận Reset Jacobian G (Solà eq 288 & 317):**
   ```
   G = eye(19);
   G(7:9, 7:9) = eye(3) - 0.5 * [dx(7:9)]x;
   P = G * P_joseph * G^T;
   ```
   Đảm bảo ma trận P luôn luôn đối xứng, xác định dương và duy trì tính nhất quán (Filter Consistency) tuyệt đối.

---

## 6. THAM SỐ KHỞI TẠO MA TRẬN HIỆP PHƯƠNG SAI BAN ĐẦU P_0

Khai báo độ bất định ban đầu để bộ lọc tự tin hội tụ mượt mà:

```text
P_0 = diag([
  (0.1)^2 * ones(1, 3),          % Vị trí ban đầu: không chắc chắn 0.1 m
  (0.01)^2 * ones(1, 3),         % Vận tốc ban đầu: không chắc chắn 0.01 m/s (sau đỗ tĩnh)
  (0.5*pi/180)^2 * ones(1, 2),   % Góc Roll, Pitch ban đầu: lệch 0.5 độ (căn chỉnh trọng lực)
  (2.0*pi/180)^2,                % Góc Heading Yaw ban đầu: lệch 2.0 độ (do gyrocompassing)
  (0.5e-3 * 9.81)^2 * ones(1,3), % Bias gia tốc ban đầu: 0.5 mg
  (0.5 * pi/180 / 3600)^2 * ones(1,3), % Bias con quay ban đầu: 0.5 deg/h
  (0.03)^2,                      % HỆ SỐ k BAN ĐẦU: Giả định k=0, độ bất định +-3% (0.03)
  (2.0 * pi/180)^2 * ones(1,3)   % BORESIGHT BAN ĐẦU: Giả định 0 độ, độ bất định +-2.0 độ
])
```

---

## 7. CÁC ĐIỂM KỸ THUẬT QUAN TRỌNG CẦN LƯU Ý KHI THỰC THI

1. **Khả năng quan sát của Roll Boresight (alpha):**
   * Do tàu chạy trên mặt phẳng ngang (v_z ≈ 0, v_y ≈ 0), trục Roll quanh trục chuyển động dọc x rất khó quan sát nếu tàu không có chuyển động lắc ngang (Roll motion).
   * **Chiến lược:** Nếu kịch bản không có sóng lắc, nên cố định alpha = 0 (hoặc đặt hiệp phương sai Q rất nhỏ), chỉ tập trung giải Pitch beta và Yaw gamma.
2. **Loại bỏ dị vật & mất khóa đáy DVL (Outlier Gating):**
   * Sử dụng khoảng cách Mahalanobis trước khi Update:
     `d2 = res^T * (H * P * H^T + R_dvl)^(-1) * res`
   * Nếu `d2 > 11.34` (ngưỡng phân bố Chi-bình phương 3 bậc tự do ở mức tin cậy 99%), bỏ qua bản tin DVL đó để bảo vệ bộ lọc không bị sốc.
3. **Khóa kênh đứng khi không có Depth Sensor:**
   * Do tàu chạy ngầm phẳng, nếu không có cảm biến áp suất đo sâu, kênh đứng sẽ có xu hướng trôi nhẹ. Có thể bổ sung ràng buộc vận tốc đứng danh định (Pseudo-measurement: `v_z_NED = 0`) với sai số `sigma = 0.05 m/s` để giữ vững cao độ.
4. **Sẵn sàng cho DSP C2000 (FPU/TMU):**
   * Toàn bộ các ma trận `F_x (19x19)`, `H (3x19)`, `P (19x19)` đều có kích thước cố định, không dùng cấp phát động `malloc`.
   * Bước nghịch đảo ma trận trong Kalman Gain chỉ là nghịch đảo ma trận `(H*P*H^T + R)` có kích thước **3x3** -> C2000 tính định thức và ma trận phụ hợp trong vòng vài micro-giây!
