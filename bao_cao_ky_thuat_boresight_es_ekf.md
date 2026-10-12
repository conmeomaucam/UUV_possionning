# BÁO CÁO KỸ THUẬT: HỆ THỐNG DẪN ĐƯỜNG TÍCH HỢP INS/DVL (BORESIGHT ES-EKF)
**Mục tiêu: Ước lượng đồng thời trạng thái chuyển động và sai số hệ thống DVL (Scale Factor & Boresight)**

---

## 1. MỤC TIÊU CỐT LÕI VÀ NHIỆM VỤ ĐỀ TÀI

Theo đề cương kỹ thuật, hệ thống tích hợp ngầm giữa IMU (INS-DM-FI) và DVL (Water Linked A125) giải quyết hai nhiệm vụ trọng tâm:

1. **Nhiệm vụ 1 (Thuật toán):** Xây dựng mô hình **ước lượng đồng thời** trạng thái chuyển động (vị trí, vận tốc, thái độ) và các sai số hệ thống:
   * Sai số hệ số tỷ lệ tốc độ (Scale Factor Error - k).
   * Sai số góc lệch trục lắp đặt (Boresight Misalignment Angles - Roll alpha, Pitch beta, Yaw gamma giữa DVL và khung thân IMU).
2. **Nhiệm vụ 2 (Nhúng thời gian thực):** Tối ưu hóa thuật toán lọc Kalman mở rộng dạng sai số (ES-EKF) để triển khai thời gian thực trên nền tảng vi điều khiển DSP C2000 (khai thác bộ đồng xử lý FPU/TMU, tối ưu thời gian chu kỳ tính toán, định dạng dữ liệu).

> [!NOTE]
> **Đặc thù bài toán:** Hệ thống hoạt động ngầm hoàn toàn, **KHÔNG CÓ CẢM BIẾN GNSS/GPS**. Toàn bộ quá trình kìm giữ sai số định vị và ước lượng tham số lắp đặt phải tự lực dựa trên sự phối hợp giữa INS và DVL.

---

## 2. PHÂN TÍCH KẾT QUẢ MÔ PHỎNG HIỆN TẠI VÀ NGUYÊN NHÂN PHÂN KỲ

### 2.1. Đánh giá kết quả trên kịch bản chạy hình số 8 (kichban_calib.m)
Trong thử nghiệm kịch bản chạy hình số 8 (4 phút) kết hợp đổi tốc độ, khi dùng phương pháp Bình phương tối thiểu (BPTTT):
* **Hệ số tỷ lệ k:** Giữ tương đối tốt trong 80s đầu, sau đó lao dốc thẳng đứng xuống `-35%` (giá trị thật là `+1.0%`) -> **Phân kỳ nặng**.
* **Pitch Boresight (beta):** Tăng dốc tuyến tính liên tục từ `0 độ` lên tới `+27 độ` sau 240s (giá trị thật là `-0.3 độ`) -> **Phân kỳ hoàn toàn**.
* **Yaw Boresight (gamma):** Dao động tuần hoàn theo chu kỳ lượn số 8 với biên độ tăng dần từ `2 độ` lên `5.5 độ` (giá trị thật là `1.0 độ`) -> **Không ổn định**.
* **Quỹ đạo (Local NED):** Bị trôi lệch trục Đông - Bắc hơn `150 m` so với điểm xuất phát.

### 2.2. Bóc tách nguyên nhân bản chất
1. **Sai lầm khi dùng Tích phân Quán tính Thuần (Pure INS) làm chuẩn:**
   * Trong vòng lặp 4 phút, INS chạy quán tính mở (không có Kalman cập nhật).
   * Bias gia tốc và con quay tích phân theo thời gian làm vận tốc INS khung thân bị trôi dạt lớn.
2. **Kênh đứng INS (Vertical Channel) không ổn định:**
   * Do không có cảm biến độ sâu (Depth Sensor) hỗ trợ, kênh đứng của INS bị trôi vận tốc đứng lên tới `v_ins_z ≈ 0.72 m/s`.
   * Phương trình BPTTT gán: `beta ≈ - v_ins_z / V`.
   * Lấy `0.72 m/s` chia cho `V = 1.5 m/s` ra góc lệch `0.48 rad` (chính là con số `27.5 độ` trên đồ thị). Toàn bộ lỗi trôi dạt INS bị hiểu nhầm thành lỗi Boresight của DVL.
3. **Không có GNSS nên không thể giải BPTTT rời rạc:**
   * BPTTT chỉ hoạt động được khi có mốc vận tốc chuẩn ngoài (như GPS khi chạy trên mặt nước).
   * Khi tàu lặn ngầm không có GPS, không thể để INS trôi dạt rồi mới trừ DVL ở cuối.

---

## 3. CƠ SỞ KHOA HỌC: MÔ HÌNH ƯỚC LƯỢNG ĐỒNG THỜI (ONLINE CO-ESTIMATION ES-EKF)

### 3.1. Các bài báo khoa học quốc tế tiêu biểu
Phương pháp ước lượng đồng thời k và Boresight bằng ES-EKF được công bố và chứng minh rộng rãi trên các tạp chí đầu ngành:
* **IEEE Sensors Journal:** *"Full-Parameters Online Calibration for SINS/DVL Integrated Navigation System"* - Chứng minh cấu trúc vector trạng thái mở rộng gộp chung trạng thái chuyển động và tham số lắp đặt DVL.
* **IEEE Transactions on Instrumentation and Measurement:** *"Observability Analysis and Calibration of SINS/DVL Misalignment and Scale Factor Error Without External Position Aiding"* - Phân tích tính quan sát được (Observability): Kịch bản lượn vòng (hình số 8) và đổi tốc độ giúp kích thích hệ thống, làm cho k và Boresight trở nên quan sát được hoàn toàn và hội tụ về giá trị thật.
* **Ocean Engineering:** *"An Augmented State Error-State Kalman Filter for High-Precision Underwater Navigation of AUV Aided by DVL"* - Mô hình toán học chi tiết của bộ lọc ES-EKF khép kín.

### 3.2. Cấu trúc Vector Trạng thái Sai số (16 trạng thái)
Bộ lọc ES-EKF làm việc trên vector trạng thái sai số mở rộng:
```
dx = [ delta_p (3x1);   % Sai số vị trí khung NED [Bắc, Đông, Xuống]
       delta_v (3x1);   % Sai số vận tốc khung NED [v_N, v_E, v_D]
       delta_theta (3x1);% Sai số góc thái độ [phi, theta, psi]
       delta_bg (3x1);  % Sai số bias con quay (Gyro bias)
       delta_ba (3x1) hoặc delta_k (1x1);
       delta_k (1x1);   % Sai số hệ số tỷ lệ DVL
       delta_phi_bs (2x1 hoặc 3x1) % Sai số góc Boresight DVL [Pitch beta, Yaw gamma] ]
```

### 3.3. Mô hình Quan sát DVL Khép kín (Closed-loop Measurement Model)
* Mô hình vận tốc DVL lý tưởng ở khung đo DVL:
  ```
  v_dvl = (1 + k) * C_b_dvl * C_n_b * v_n
  ```
  Trong đó `C_b_dvl = eye(3) - [phi_bs x]` là ma trận chuyển từ khung thân sang khung DVL.
* **Quy trình hoạt động thời gian thực:**
  * **Chu kỳ 400 Hz (IMU):** Tích phân INS cập nhật trạng thái danh định.
  * **Chu kỳ 5 Hz (DVL - mỗi 0.2s):** 
    * Tính phần dư: `residual = z_dvl - h(x_danh_dinh)`.
    * Trong 0.2s, INS chưa kịp trôi dạt (sai số trôi < 0.0001 m/s).
    * Tính Kalman Gain K và cập nhật vector trạng thái sai số `dx`.
    * **Feedback ngay lập tức:** Sửa lại vận tốc và góc cho INS danh định, đồng thời reset `dx` về 0.
    * Tích lũy giá trị ước lượng của `k` và `Boresight`.

---

## 4. BẢNG TỔNG KẾT CÁC FILE TRONG DỰ ÁN

| File | Trạng thái | Vai trò & Đánh giá |
| :--- | :---: | :--- |
| **thong_so_he_thong_chuan.m** | Chuẩn | Lưu thông số phần cứng INS-DM-FI, DVL A125, giá trị thật (k = 1%, Boresight = [0.2, -0.3, 1.0] deg). |
| **mo_phong_sinh_du_lieu_full.m**| Chuẩn | Sinh dữ liệu IMU 400 Hz (có bias, ARW, VRW) và DVL 5 Hz theo mô hình vật lý chính xác. |
| **dvl_do_khung_than.m** | Chuẩn | Hàm đo vận tốc DVL từ vận tốc NED và Quaternion thái độ. |
| **kichban_calib.m** | Cần nâng cấp | Hiện đang giải BPTTT rời rạc trên Pure INS nên bị phân kỳ. Cần chuyển đổi sang thuật toán ES-EKF khép kín. |
| **giai_k_boresight_uoc_luong.m**| Tham khảo | Thử nghiệm BPTTT trong pha chạy thẳng ngắn (đã ghi chú sai số do INS gây ra). |
| **bao_cao_ky_thuat_boresight_es_ekf.md** | Hoàn thiện | Tài liệu kỹ thuật tổng hợp toàn bộ lý thuyết, phân tích lỗi và lộ trình triển khai. |

---

## 5. KẾ HOẠCH VÀ CÁC BƯỚC THỰC HIỆN TIẾP THEO

### Bước 1: Xây dựng Hàm Quan sát Jacobian H cho DVL
* Xây dựng ma trận đạo hàm riêng của số đo DVL đối với các biến trạng thái:
  * Đạo hàm theo vận tốc: `H_v = (1 + k) * C_b_dvl * C_n_b`.
  * Đạo hàm theo góc thái độ: `H_theta = (1 + k) * C_b_dvl * C_n_b * [v_n x]`.
  * Đạo hàm theo hệ số tỷ lệ: `H_k = C_b_dvl * C_n_b * v_n`.
  * Đạo hàm theo góc Boresight: `H_bs = (1 + k) * [ (C_n_b * v_n) x ]`.

### Bước 2: Triển khai Kịch bản ES-EKF 16 trạng thái (chay_es_ekf_boresight.m)
* Viết script chạy vòng lặp tích hợp đầy đủ:
  * Tích phân cơ học INS 400 Hz.
  * Dự báo hiệp phương sai P (Prediction step).
  * Cập nhật đo lường DVL 5 Hz (Update step).
  * Feedback sửa sai số cho INS và cập nhật trực tuyến k, Boresight.
* Đặt giá trị khởi tạo danh định: `k = 0`, `Boresight = [0, 0, 0]`.

### Bước 3: Đánh giá Hội tụ và Kiểm tra Tính Bền vững
* Chạy kịch bản Hình số 8 (4 phút) qua bộ lọc ES-EKF.
* Vẽ đồ thị hội tụ của:
  * Hệ số k (kiểm tra hội tụ về đúng 1.0%).
  * Pitch beta (kiểm tra hội tụ về đúng -0.3 độ).
  * Yaw gamma (kiểm tra hội tụ về đúng 1.0 độ).
  * Sai số vị trí và vận tốc (đảm bảo không còn bị dạt 150m như trước).

### Bước 4: Chuẩn bị Cấu trúc Thuật toán cho DSP C2000 (Nhiệm vụ 2)
* Tối ưu kích thước ma trận (tận dụng tính đối xứng của ma trận P).
* Thiết kế thuật toán theo dạng bảng dữ liệu tĩnh, tránh cấp phát bộ nhớ động.
* Tách biệt các phép tính lượng giác và căn bậc hai để sẵn sàng gọi tập lệnh phần cứng TMU/FPU trên C2000.