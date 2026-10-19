# TỔNG QUAN KIẾN TRÚC VÀ THUẬT TOÁN HỆ THỐNG DẪN ĐƯỜNG UUV ES-EKF 19 TRẠNG THÁI

> **Tài liệu tổng hợp kiến trúc hệ thống, cơ sở lý thuyết, luồng thuật toán và cấu trúc mã nguồn trong workspace.**

---

## 1. Mục tiêu cốt lõi của dự án

Dự án tập trung nghiên cứu, xây dựng và đánh giá thuật toán **Dẫn đường tích hợp quán tính INS/DVL/GNSS/Depth** cho phương tiện lặn tự hành dưới nước (**UUV - Unmanned Underwater Vehicle**), giải quyết trọn vẹn bài toán **hiệu chuẩn trực tuyến (Online Calibration)**:

1. **Hiệu chuẩn 3 góc lệch lắp đặt cơ khí (Boresight angles):** Góc lệch giữa khung đầu đo DVL và khung thân IMU/INS gồm $[\alpha, \beta, \gamma]$ (Roll, Pitch, Yaw).
2. **Ước lượng Hệ số tỉ lệ co giãn chùm tia DVL (Scale factor $s$):** Bù sai số vận tốc đáy do biến thiên môi trường âm học (nhiệt độ, độ mặn, áp suất nước biển).
3. **Ước lượng và khử trôi Bias IMU thời gian thực:** Tách và trừ khử độ trôi con quay hồi chuyển ($\mathbf{b}_g$) và gia tốc kế ($\mathbf{b}_a$ theo mô hình Gauss-Markov bậc 1).
4. **Bù hiệu ứng cánh tay đòn (Lever arm):** Bù vận tốc cảm ứng do khoảng cách lệch tâm giữa vị trí IMU và DVL ($\mathbf{l}_{\text{arm}} = [0.2, 0, -0.1]\text{ m}$).
5. **Dẫn đường chính xác khi lặn ngầm mất GNSS:** Đảm bảo khi UUV ngắt hoàn toàn GNSS tại thời điểm $T_{\text{cal}}$ và lặn sâu, sai số tích lũy vị trí vẫn giữ ở mức cực nhỏ (vài chục cm trên hành trình hàng trăm mét).

---

## 2. Kiến trúc thuật toán Bộ lọc ES-EKF 19 trạng thái

Bộ lọc được thiết kế theo cấu trúc **Error-State Extended Kalman Filter (ES-EKF)** gồm 2 tầng tốc độ lấy mẫu:

```
                  ┌────────────────────────────────────────────────────────┐
                  │          TẦNG DỰ ĐOÁN QUÁN TÍNH INS (400 Hz)           │
                  │  (Tích phân vận tốc, vị trí, quaternion, lan truyền P)  │
                  └──────────────────────────┬─────────────────────────────┘
                                             │ Trạng thái danh định x (20x1)
                                             ▼
┌───────────────────────┐            ┌───────────────┐            ┌───────────────────────┐
│ ĐẦU VÀO NGOẠI VI 5 Hz │ ── đo z ─► │ CỔNG LỌC NIS  │ ── r, H ─► │ CẬP NHẬT KALMAN 5 Hz  │
│ (GNSS / DVL / Depth)  │            │ (Chi-square)  │            │ (Ước lượng δx, tính K)│
└───────────────────────┘            └───────────────┘            └──────────┬────────────┘
                                                                             │ Vector sai số δx (19x1)
                                                                             ▼
                                                                  ┌───────────────────────┐
                                                                  │ BƠM SAI SỐ (INJECTION)│
                                                                  │  x = x ⊕ δx;  δx = 0  │
                                                                  └──────────┬────────────┘
                                             ┌───────────────────────────────┴─────────────────┐
                                             ▼                                                 ▼
                              [HỒI TIẾP 1: Trạng thái chuẩn]                    [HỒI TIẾP 2: Khử trôi Bias]
                              Làm gốc cho chu kỳ dự đoán mới                     Trừ trực tiếp đầu vào IMU
```

### 2.1. Vector trạng thái danh định (Nominal State - 20 chiều)
$$\mathbf{x} = \begin{bmatrix} \mathbf{p}^T & \mathbf{v}^T & \mathbf{q}^T & \mathbf{b}_a^T & \mathbf{b}_g^T & s & \boldsymbol{\phi}_{\text{bs}}^T \end{bmatrix}^T \in \mathbb{R}^{20}$$
- $\mathbf{p} = [p_N, p_E, p_D]^T$: Vị trí trong hệ tọa độ NED (m).
- $\mathbf{v} = [v_N, v_E, v_D]^T$: Vận tốc trong hệ tọa độ NED (m/s).
- $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$: Quaternion biểu diễn tư thế UUV.
- $\mathbf{b}_a = [b_{ax}, b_{ay}, b_{az}]^T$: Bias gia tốc kế trong khung thân (m/s²).
- $\mathbf{b}_g = [b_{gx}, b_{gy}, b_{gz}]^T$: Bias con quay trong khung thân (rad/s).
- $s$: Hệ số tỉ lệ co giãn của DVL (không thứ nguyên).
- $\boldsymbol{\phi}_{\text{bs}} = [\alpha, \beta, \gamma]^T$: 3 góc lệch Boresight DVL (rad).

### 2.2. Vector trạng thái sai số (Error State - 19 chiều)
$$\delta\mathbf{x} = \begin{bmatrix} \delta\mathbf{p} & \delta\mathbf{v} & \delta\boldsymbol{\theta} & \delta\mathbf{b}_a & \delta\mathbf{b}_g & \delta s & \delta\boldsymbol{\phi}_{\text{bs}} \end{bmatrix}^T \in \mathbb{R}^{19}$$
*(Trong đó $\delta\boldsymbol{\theta} \in \mathbb{R}^3$ là vector góc quay sai số tối thiểu, tránh suy biến số học của quaternion 4 chiều).*

### 2.3. Hai tầng vận hành và 2 vòng hồi tiếp kín
1. **Tầng 1 - Dự đoán quán tính INS tốc độ cao (400 Hz):**
   - Đọc gia tốc riêng $\mathbf{a}_{\text{meas}}$ và vận tốc góc $\boldsymbol{\omega}_{\text{meas}}$.
   - Trừ bias đã ước lượng: $\mathbf{a}_{\text{corr}} = \mathbf{a}_{\text{meas}} - \mathbf{b}_a$; $\boldsymbol{\omega}_{\text{corr}} = \boldsymbol{\omega}_{\text{meas}} - \mathbf{b}_g$.
   - Tích phân phương trình động học chuyển động trong hệ NED (bù hiệu ứng Trái Đất $\boldsymbol{\omega}_{ie}, \boldsymbol{\omega}_{en}$ và trọng trường $\mathbf{g}$).
   - Lan truyền ma trận hiệp phương sai sai số: $\mathbf{P} = \mathbf{F}\mathbf{P}\mathbf{F}^T + \mathbf{Q}$.
   - Suy giảm bias gia tốc theo mô hình Gauss-Markov: $\mathbf{b}_a = \exp(-\Delta t/\tau) \mathbf{b}_a$ ($\tau = 100\text{ s}$).

2. **Tầng 2 - Cập nhật đo lường ngoại vi tốc độ thấp (5 Hz):**
   - **GNSS RTK + Compass (7 chiều):** Cập nhật vị trí $[\mathbf{p}]$, vận tốc $[\mathbf{v}]$, và góc hướng $[\psi]$ khi UUV bơi mặt nước.
   - **Cảm biến độ sâu (1 chiều):** Đo trực tiếp tọa độ trục Down $p_D$ toàn thời gian.
   - **DVL A125 (3 chiều):** Đo vận tốc khung đầu dò DVL. Hàm đo phi tuyến có bù cánh tay đòn và góc Boresight:
     $$\mathbf{h}(\mathbf{x}) = (1+s) \mathbf{C}_b^s \big[\mathbf{C}_n^b \mathbf{v} + \boldsymbol{\omega}_{nb}^b \times \mathbf{l}_{\text{arm}}\big]$$
   - **Kiểm định cổng thống kê $\chi^2$ (NIS):**
     $$\text{NIS} = \mathbf{r}^T \mathbf{S}^{-1} \mathbf{r} \le \gamma_{\text{threshold}}$$
     *(Ngưỡng 99%: GNSS $\le 18.48$, Depth $\le 6.63$, DVL $\le 11.34$. Phép đo vượt ngưỡng sẽ bị từ chối để loại bỏ ngoại lai).*
   - **Cập nhật hiệp phương sai chuẩn Joseph (ổn định số học):**
     $$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$$

3. **Bơm sai số (Error Injection) và Đặt lại:**
   - Bơm $\delta\mathbf{x}$ trực tiếp vào $\mathbf{x}$: $\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}$; $\mathbf{v} \leftarrow \mathbf{v} + \delta\mathbf{v}$; $\mathbf{q} \leftarrow \mathbf{q} \otimes [1, \frac{1}{2}\delta\boldsymbol{\theta}]^T$; $s \leftarrow s + \delta s$; $\boldsymbol{\phi} \leftarrow \boldsymbol{\phi} + \delta\boldsymbol{\phi}$.
   - Đặt lại vector sai số $\delta\mathbf{x} = \mathbf{0}$, cập nhật lại ma trận $\mathbf{P}$.

4. **Hai vòng hồi tiếp kín (Closed-loop Feedback):**
   - **Hồi tiếp 1:** Đưa trạng thái chuẩn vừa hiệu chỉnh làm điểm xuất phát mới cho chu kỳ dự đoán INS 400 Hz tiếp theo, triệt tiêu việc tích lũy sai số trôi dạt.
   - **Hồi tiếp 2:** Đưa bias ước lượng ($\hat{\mathbf{b}}_a, \hat{\mathbf{b}}_g$) trực tiếp về cổng trừ đầu vào IMU để khử trôi vật lý trước khi giải động học.

---

## 3. Cấu hình phần cứng và tham số mô phỏng chuẩn

| Cảm biến / Thành phần | Tham số phần cứng mô phỏng | Xử lý trong bộ lọc ES-EKF |
| :--- | :--- | :--- |
| **INS-DM-FI (IMU-FI-200T)**<br>*(Tần số: 400 Hz)* | • Gyro ARW: $0.025^\circ/\sqrt{\text{h}}$, bias tĩnh $0.5^\circ/\text{h}$, scale $100\text{ ppm}$<br>• Accel VRW: $0.015\text{ m/s}/\sqrt{\text{h}}$, bias $0.5\text{ mg}$, trôi GM $0.005\text{ mg}$ ($100\text{ s}$) | • Ước lượng 3 bias con quay, 3 bias gia tốc<br>• Đưa ARW, VRW và trôi GM vào ma trận $\mathbf{Q}$ |
| **DVL A125**<br>*(Tần số: 5 Hz)* | • Chuẩn góc Boresight: $[\alpha, \beta, \gamma] = [0.2^\circ, -0.3^\circ, 1.0^\circ]$<br>• Chuẩn Scale: $1.0000\%$ ($k_{\text{true}} = 0.01$)<br>• Cánh tay đòn: $\mathbf{l}_{\text{arm}} = [0.2, 0, -0.1]\text{ m}$<br>• Nhiễu trắng: $5\text{ mm/s} + 0.2\% |\mathbf{v}|$ | • Ước lượng Scale $s$ và 3 góc Boresight<br>• Bù động học cánh tay đòn $\boldsymbol{\omega} \times \mathbf{l}_{\text{arm}}$<br>• Cổng lọc NIS $\le 11.34$ |
| **GNSS RTK & Compass**<br>*(Tần số: 5 Hz)* | • Bật khi $t \le T_{\text{cal}}$ (ở mặt nước)<br>• Nhiễu vị trí: $0.02\text{ m}$; Vận tốc: $0.02\text{ m/s}$; Yaw: $0.05^\circ$ | • Cập nhật 7 chiều $[\mathbf{p}, \mathbf{v}, \psi]$<br>• Cổng lọc NIS $\le 18.48$ |
| **Cảm biến độ sâu**<br>*(Tần số: 5 Hz)* | • Hoạt động toàn thời gian<br>• Nhiễu trắng trục Down: $0.05\text{ m}$ | • Cập nhật trực tiếp $p_D$<br>• Cổng lọc NIS $\le 6.63$ |

---

## 4. Hệ thống 6 kịch bản thử nghiệm (`roll_boresight.m`)

Đặc tính quan sát: Góc lệch Pitch ($\beta$) và Yaw ($\gamma$) có thể quan sát tốt khi tàu đổi hướng hoặc bơi tiến. Riêng góc lệch Roll ($\alpha$) đòi hỏi tàu phải có **kích thích chuyển động lắc ngang (Sway) và lắc Roll** mới bộc lộ sai số:

| Kịch bản | Tên kịch bản | $T_{\text{cal}}$ | Đặc điểm quỹ đạo & Mức độ kích thích | Đích đến NED | Ý nghĩa thử nghiệm |
| :---: | :--- | :---: | :--- | :---: | :--- |
| **Case 1** | *Baseline turn & dive* | 250 s | Sway $0.18\text{ m/s}$, Roll $10^\circ$, vận tốc $1.2\text{ m/s}$ | $[450; 250; 15]$ m | Kịch bản đối chứng chuẩn |
| **Case 2** | *Aggressive maneuver* | 220 s | Sway $0.24\text{ m/s}$, Roll $12^\circ$, vận tốc $1.5\text{ m/s}$ | $[520; -140; 20]$ m | Kiểm tra độ ổn định khi cơ động mạnh |
| **Case 3** | *Mild excitation* | 280 s | Sway $0.12\text{ m/s}$, Roll $6^\circ$, vận tốc $1.0\text{ m/s}$ | $[350; 420; 10]$ m | Đánh giá khi kích thích biên độ nhỏ |
| **Case 4** | *Strong excitation* | 200 s | Sway $0.28\text{ m/s}$, Roll $15^\circ$, vận tốc $1.7\text{ m/s}$ | $[650; 150; 25]$ m | Kích thích mạnh nhất; góc $\alpha$ hội tụ tốt nhất |
| **Case 5** | *Low excitation* | 260 s | Sway $0.05\text{ m/s}$, Roll $3^\circ$, vận tốc $1.2\text{ m/s}$ | $[400; 200; 12]$ m | Thử nghiệm ứng suất cực hạn (góc $\alpha$ khó quan sát) |
| **Case 6** | *Lemniscate (Vòng số 8)* | 250 s | Bơi vòng số 8; nổi lên mặt nước lúc $700\text{ s}$ | Gốc $[0; 0; 0]$ m | Đo độ trôi thực tế lúc nổi (% DT và m/giờ) |

---

## 5. Danh mục và vai trò các file trong Workspace

```
BORESIGHT_ES_EKF/
├── TONG_QUAN_HE_THONG_UUV_ESEKF.md   # [FILE NÀY] Tài liệu tổng quan kiến trúc và tra cứu toàn bộ dự án
│
├── [CỐT LÕI THUẬT TOÁN ES-EKF 19 TRẠNG THÁI]
│   ├── esekf19_init.m                # Khởi tạo trạng thái danh định x (20x1), hiệp phương sai P (19x19) và Q
│   ├── esekf19_step.m                # Trái tim bộ lọc: Tích phân 400Hz, cập nhật GNSS/DVL/Depth, lọc NIS, Joseph
│   ├── dvl_do_khung_than.m           # Chuyển đổi mô hình vận tốc DVL và hình học chùm tia
│   └── thong_so_he_thong_chuan.m     # Cấu hình phần cứng và hằng số Trái Đất (WGS-84, g, bán kính cong)
│
├── [ĐIỀU KHIỂN MÔ PHỎNG VÀ VẼ ĐỒ THỊ MATLAB]
│   ├── roll_boresight.m              # Script chạy mô phỏng 6 kịch bản, tính Benchmark sai số lúc nổi
│   ├── roll_boresight_plot.m         # Vẽ biểu đồ hội tụ Boresight, sai số vị trí, ellipse độ bất định ±3σ
│   ├── chay_es_ekf_boresight.m       # Script điều phối tích hợp và xuất kết quả nghiệm thu
│   ├── dithang.m                     # Kịch bản kiểm chứng cơ bản khi tàu chỉ chạy thẳng
│   ├── mo_phong_3d_matlab.m          # Minh họa quỹ đạo chuyển động 3D không gian
│   └── ve_do_thi_hoi_tu.m            # Đồ thị so sánh tốc độ hội tụ của các tham số
│
├── [TẠO BÁO CÁO VÀ HÌNH ẢNH KỸ THUẬT]
│   ├── generate_report.py            # Code Python sinh ảnh sơ đồ 300 DPI và xuất file Word báo cáo chuẩn (.docx)
│   ├── So_Do_Khoi_ES_EKF_19.png      # Ảnh sơ đồ khối kỹ thuật 2 tầng 2 vòng hồi tiếp
│   ├── Case1_Hinh_1..4.png           # 4 hình kết quả kịch bản 1 (Quỹ đạo 3D, Boresight, Sai số, NIS & Bias)
│   ├── Case6_QuyDao_So8.png          # Đồ thị quỹ đạo vòng số 8 kịch bản 6
│   └── Bao_Cao_Mo_Phong_UUV_Kem_So_Do_Arial14.docx # Báo cáo hoàn chỉnh nghiệm thu kỹ thuật (Font Arial 14)
│
└── [.gitignore]                      # Cấu hình bỏ qua các file dữ liệu mô phỏng lớn (*.mat > 100MB) khi push Git
```

---

## 6. Hướng dẫn vận hành nhanh

### 6.1. Chạy mô phỏng trên MATLAB
```matlab
% Chạy kịch bản 1 cơ bản và vẽ đồ thị
result1 = roll_boresight(1, true, true);

% Chạy kịch bản 6 (Vòng số 8: Nổi - Lặn - Nổi nghiệm thu)
result6 = roll_boresight(6, true, true);
```

### 6.2. Xuất báo cáo Word kèm sơ đồ khối và hình ảnh
```powershell
python generate_report.py
```
*Kết quả xuất ra file Word: `Bao_Cao_Mo_Phong_UUV_Kem_So_Do_Arial14.docx` (kèm cơ chế tự động chuyển sang `..._moi.docx` nếu file gốc đang mở trong Word).*

### 6.3. Đẩy code lên GitHub (sau khi cập nhật)
```powershell
git add .
git commit -m "noi dung cap nhat"
git push origin main
```
