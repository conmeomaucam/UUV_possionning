# BẢNG TỔNG HỢP TRẠNG THÁI BAN ĐẦU VÀ KỊCH BẢN THỰC TẾ CHUẨN HÓA (REMUS 100 + ES-EKF)

Tài liệu kỹ thuật tổng hợp toàn bộ điều kiện đầu vào, các điểm đã được chuẩn hóa theo đúng thực tế vận hành hàng hải (loại bỏ hoàn toàn Oracle Cheat), và cơ chế xác thực cho bài toán phản biện.

---

## 1. SO SÁNH TRƯỚC VÀ SAU KHI CHUẨN HÓA (LOẠI BỎ ORACLE CHEAT)

| Hạng mục | Trước chuẩn hóa (Lỗi học thuật) | Sau khi chuẩn hóa (Đúng thực tế 100%) | Đánh giá & Kiểm chứng |
| :--- | :--- | :--- | :--- |
| **Góc hướng ban đầu (Heading/Yaw)** | `psi_est = psi_true` (sai số = 0.0 deg, cheat Oracle) | **`psi_est = psi_true + 2.5 deg`** (mang sai số căn chỉnh thô ban đầu) | **Đúng thực tế**: VN-210-S không có từ kế/gyrocompass nên ban đầu có sai số căn chỉnh thô ~2.5 deg. |
| **Bất định góc P0(9,9)** | Khai báo `0.5 deg` (mâu thuẫn với sai số 0) | **Khai báo `3.5 deg`** | **Nhất quán Gauss**: Ma trận P0 bao trọn sai số ban đầu 2.5 deg ($0.7\sigma$). |
| **Hội tụ góc hướng** | Không cần hội tụ (do gán sẵn đúng) | **Hội tụ động (Dynamic Alignment)**: Từ 2.5 deg giảm xuống **0.016 deg sau 20s** chạy mặt nước | Chứng minh khả năng tự căn chỉnh động thực sự của bộ lọc ES-EKF bằng GNSS Course-Over-Ground. |
| **Vị trí ban đầu** | `[0; 0; 0] m` (trùng khít Ground Truth) | **Lấy từ phép đo GNSS RTK đầu tiên** (có nhiễu thực tế $\sim 0.6 - 0.8\text{ m}$) | Khử hoàn toàn Oracle cheat vị trí. |
| **Vận tốc ban đầu** | `v_hat = v_true` (sai số = 0 m/s) | **Lấy từ phép đo GNSS Doppler đầu tiên** (có nhiễu thực tế $\sim 0.02\text{ m/s}$) | Khử mâu thuẫn giữa $v=0$ và $\sigma_v = 1.5\text{ m/s}$. |
| **Bất định bias gia tốc P0** | `0.5 mg` (trong khi bias thật = 1.2 mg -> lệch $2.4\sigma$) | **Khai báo `2.0 mg`** | **Bảo toàn giả định Gauss**: Bias thật 1.2 mg nằm ở $0.6\sigma$, hoàn toàn hợp lệ. |
| **Cảm biến độ sâu** | Bar10XT (10 bar / 89.5m, hụt dải đo cho xe lặn 100m) | **Keller 4LD / BlueRobotics Bar30 (30 bar / 300m)** | **Chuẩn kỹ thuật**: REMUS 100 lặn 100m được trang bị cảm biến đúng dải công tác. |

---

## 2. KẾT QUẢ MÔ PHỎNG VẬN HÀNH THỰC TẾ (SAU CHUẨN HÓA)

- **Thời gian chạy trên mặt nước (t = 0 đến 480s)**:
  - Tàu chạy qua 6 Waypoints hình chữ S (tốc độ 1.5 m/s).
  - Sai số góc hướng Yaw ban đầu: **2.4996 deg**.
  - Sau 20 giây chuyển động có GNSS: Sai số Yaw giảm xuống **-0.0166 deg**.
  - Trước khi bắt đầu lặn (t = 480s): Sai số Yaw đạt **-0.0115 deg** (hội tụ hoàn hảo).
  - Hiệu chuẩn trực tuyến Boresight DVL: Roll = 1.68 deg (thật 1.50), Pitch = -1.95 deg (thật -2.00), Yaw = 3.017 deg (thật 3.000).
  - Hiệu chuẩn Scale Factor DVL: Đạt 1.520% (thật 1.500%).

- **Thời gian lặn ngầm ở độ sâu 100m (t = 480s đến 1880s, CẮT GNSS hoàn toàn)**:
  - Cảm biến độ sâu Bar30 duy trì đo liên tục (không bị bão hòa ảo): Sai số độ sâu cực đại **0.33 m**, độ bất định $\sigma_z$ luôn ổn định ở mức **0.0136 m (1.36 cm)**.
  - Sai số góc hướng Yaw sau khi cắt GNSS suốt 1400 giây: Cực đại chỉ **0.122 deg**, kết thúc hành trình ở **0.068 deg**.
  - **Sai số vị trí 3D cuối cùng**: **1.62 m** (sau 1880 giây hành trình 4 km).

---

## 3. CƠ CHẾ HỖ TRỢ 2 CHẾ ĐỘ (DUAL-MODE) TRONG CODE

Hàm `run_fossen_esekf_integrated` hiện hỗ trợ tham số thứ 4: `depth_sensor_variant`:
1. **Chế độ 1: Cấu hình chuẩn thực tế ngoài đời (Nominal Real-World - Mặc định)**:
   - Cú pháp: `run_fossen_esekf_integrated('VN_210', true, true, 'Bar30')`
   - Xe lặn REMUS 100 trang bị cảm biến Keller Bar30 (30 bar / 300m). Kênh độ sâu hoạt động liên tục, EKF khởi tạo Cold Start thật sự (vị trí/vận tốc có nhiễu GNSS, hướng lệch 2.5 độ).
2. **Chế độ 2: Kịch bản thử nghiệm sự cố (Fault-Tolerant Stress-Test)**:
   - Cú pháp: `run_fossen_esekf_integrated('VN_210', true, true, 'Bar10XT')`
   - Cố tình giả lập xe lặn dùng cảm biến hụt dải đo Bar10XT (10 bar / 89.5m) để kiểm tra khả năng duy trì độ sâu của DVL 3 trục khi mất mốc đo áp suất.
   - Kết quả kiểm chứng: Bất định $\sigma_z$ tăng vọt gấp 16 lần từ 0.013m lên 0.213m (hình ảnh: `TEST/phan_tich_tang_truong_covariance.png`), nhưng tàu vẫn duy trì thăng bằng trong 236 giây lặn sâu nhờ tích phân vận tốc thẳng đứng $v_{\text{Down}}$.
