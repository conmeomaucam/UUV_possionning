# BÁO CÁO KỸ THUẬT NGHIỆM THU ĐỀ TÀI
## HỆ THỐNG DẪN ĐƯỜNG TÍCH HỢP INS/GNSS/DVL 19 TRẠNG THÁI HIỆU CHUẨN TRỰC TUYẾN BORESIGHT VÀ SCALE FACTOR CHO XE LẶN TỰ HÀNH REMUS 100

---

## MỤC LỤC TỔNG QUAN BÁO CÁO

- **LỜI MỞ ĐẦU & TÓM TẮT DỰ ÁN**
- **CHƯƠNG 1: TỔNG QUAN HỆ THỐNG VÀ CÁC CHỈ TIÊU KỸ THUẬT (KPIs)**
  - 1.1. Bối cảnh bài toán dẫn đường xe lặn UUV/AUV khi mất tín hiệu GNSS
  - 1.2. Bảng 5 chỉ tiêu kỹ thuật nghiệm thu bắt buộc
  - 1.3. Cấu hình phần cứng cảm biến chuẩn công nghiệp (VN-210-S, DVL A125, Bar10XT)
- **CHƯƠNG 2: MÔ HÌNH TOÁN HỌC ĐỘNG HỌC VÀ CẢM BIẾN XE LẶN**
  - 2.1. Động lực học phi tuyến 6-DOF REMUS 100 theo chuẩn Fossen (12 trạng thái)
  - 2.2. Mô hình sai số cảm biến IMU (ARW, VRW, In-run Bias Gauss-Markov)
  - 2.3. Mô hình hình học lắp đặt DVL: Cánh tay đòn Lever-Arm, Boresight và Scale Factor
  - 2.4. Phân tích hiện tượng từ trường vỏ tàu và quy trình chuẩn hóa từ kế HSI
- **CHƯƠNG 3: THIẾT KẾ THUẬT TOÁN ĐỊNH VỊ VÀ ĐIỀU KHIỂN KHÉP VÒNG**
  - 3.1. Thiết lập vector sai số trạng thái 19 chiều (ES-EKF 19)
  - 3.2. Khâu lan truyền trạng thái và ma trận hiệp phương sai sai số ở 100 Hz
  - 3.3. Khâu cập nhật đo đạc đa tần số (GNSS RTK 5 Hz, Áp kế 5 Hz, DVL 5 Hz, Từ kế 5 Hz)
  - 3.4. Cơ chế khóa hiệu chuẩn (Calibration Lock) khi chuyển tiếp pha lặn ngầm
  - 3.5. Hệ thống dẫn đường ALOS3D và điều khiển trượt thích nghi Integral SMC
- **CHƯƠNG 4: XÂY DỰNG MÔ HÌNH MÔ PHỎNG TÍCH HỢP TRÊN SIMULINK**
  - 4.1. Cấu trúc kiến trúc mô hình `remus100_esekf_integrated.slx`
  - 4.2. Thiết kế 4 khối S-Function Level-2 (Plant, Sensors, ES-EKF, Control)
  - 4.3. Giải pháp triệt tiêu vòng lặp đại số (Algebraic Loop Elimination)
- **CHƯƠNG 5: KẾT QUẢ THỰC NGHIỆM MÔ PHỎNG VÀ ĐÁNH GIÁ ĐỊNH LƯỢNG**
  - 5.1. Bảng đối chiếu định lượng 5 chỉ tiêu kỹ thuật KPI (Nghiệm thu ĐẠT 100%)
  - 5.2. Phân tích Chi tiết 4 Đồ thị Nghiệm thu
    - Hình 1: Quỹ đạo không gian 3D và bám 12 Waypoints
    - Hình 2: Quá trình hội tụ 3 góc lệch Boresight DVL (Roll, Pitch, Yaw)
    - Hình 3: Quá trình hội tụ hệ số tỉ lệ tốc độ DVL Scale Factor
    - Hình 4: Sai số vị trí 3D theo thời gian và quãng đường di chuyển
  - 5.3. Phân tích nguyên nhân ảnh hưởng góc hướng từ kế và bài học kỹ thuật
- **CHƯƠNG 6: KẾT LUẬN VÀ KIẾN NGHỊ PHÁT TRIỂN**

---

# CHI TIẾT NỘI DUNG TỪNG PHẦN ĐỂ ĐƯA VÀO BÁO CÁO

## CHƯƠNG 1: TỔNG QUAN HỆ THỐNG VÀ CÁC CHỈ TIÊU KỸ THUẬT (KPIs)

### 1.1. Bối cảnh kỹ thuật
Tàu lặn tự hành (AUV/UUV) hoạt động trong môi trường biển sâu, nơi sóng điện từ tần số cao bị nước biển suy hao triệt để, khiến tín hiệu GNSS hoàn toàn không thể thu nhận khi lặn sâu quá 0.5 m. Hệ dẫn đường quán tính thuần (Free Inertial Navigation) sử dụng IMU cấp chiến thuật tích lũy sai số vị trí theo hàm bậc 3 thời gian, nhanh chóng phân kỳ hàng trăm mét chỉ sau vài phút. 

Để đảm bảo hành trình dài 7 km, giải pháp dẫn đường bắt buộc phải là sự kết hợp Loosely-Coupled giữa:
- Cụm đo quán tính IMU tần số cao 100 Hz.
- Cảm biến đo vận tốc đáy biển bằng sóng âm DVL (Doppler Velocity Log) 5 Hz.
- Cảm biến áp suất đo độ sâu Barometer 5 Hz.
- Cảm biến từ kế xác định góc hướng la bàn 5 Hz.
- Hệ thống định vị vệ tinh GNSS RTK ở pha nổi ban đầu để căn chỉnh và hiệu chuẩn trực tuyến.

Tuy nhiên, trong thực tế chế tạo và lắp đặt cơ khí, trục đo của DVL luôn bị lệch một góc không gian so với hệ trục thân tàu (góc Boresight), đồng thời sự thay đổi vận tốc âm trong nước biển gây ra sai số tỉ lệ tốc độ (Scale Factor). Nếu không được ước lượng và bù trừ trực tuyến, sai số này sẽ làm sai lệch hoàn toàn phép chiếu vận tốc DVL, dẫn đến sai số vị trí tích lũy nghiêm trọng khi lặn ngầm.

### 1.2. Bảng 5 chỉ tiêu kỹ thuật nghiệm thu bắt buộc (KPIs)
Báo cáo cần trình bày rõ bảng tiêu chí kỹ thuật do đơn vị giao đề tài (Viettel / Hội đồng) quy định:

| STT | Tên chỉ tiêu kỹ thuật | Yêu cầu kỹ thuật | Kết quả đạt được | Đánh giá |
|:---:|:---|:---:|:---:|:---:|
| 1 | Tần số xử lý chu kỳ dẫn đường | >= 100 Hz (dt <= 10 ms) | 100 Hz (h = 0.01 s) | **ĐẠT** |
| 2 | Sai số vị trí tích lũy Dead-Reckoning ngầm | < 1.0% Tổng quãng đường (% DT) | ~0.25% DT (~18 - 25 m / 7.2 km) | **ĐẠT** |
| 3 | Sai số hiệu chuẩn trực tuyến góc Boresight DVL | < 0.6 độ trên cả 3 trục (R, P, Y) | Roll: 0.012° \| Pitch: 0.005° \| Yaw: 0.045° | **ĐẠT** |
| 4 | Sai số ước lượng hệ số tỉ lệ DVL Scale Factor | < 1.0% | 0.021% (Ước lượng 1.521% vs Thật 1.500%) | **ĐẠT** |
| 5 | Bộ sản phẩm bàn giao | Mã nguồn, Simulink, Dữ liệu, 4 Đồ thị | Đầy đủ 100% | **ĐẠT** |

### 1.3. Cấu hình phần cứng cảm biến theo Datasheet
- **IMU/GNSS VectorNav VN-210-S (Tactical-Grade):**
  - Con quay Gyroscope: In-run Bias Stability = 1.0 deg/h; Angular Random Walk (ARW) = 0.042 deg/sqrt(h) = 0.0007 deg/sqrt(s).
  - Gia tốc kế Accelerometer: In-run Bias Stability = 10 u-g; Velocity Random Walk (VRW) = 0.024 m/s/sqrt(h) = 0.0004 m/s/sqrt(s).
  - GNSS RTK: Sai số mặt phẳng nằm ngang = 0.01 m, đứng = 0.02 m, sai số vận tốc = 0.02 m/s.
- **DVL Nortek Nucleus A125:**
  - Tần số phát âm: 1 MHz, tầm đo đáy 0.2 m đến 125 m.
  - Sai số đo vận tốc: 0.15% vận tốc đo + 0.002 m/s (noise floor). Tần số cập nhật: 5 Hz.
- **Áp kế đo sâu BlueRobotics Bar10XT:**
  - Dải áp suất 10 bar (độ sâu tối đa 100 m), độ phân giải 0.2 mbar (~2 mm nước), sai số danh định 0.08 m. Tần số cập nhật: 5 Hz.

---

## CHƯƠNG 2: MÔ HÌNH TOÁN HỌC ĐỘNG HỌC VÀ CẢM BIẾN XE LẶN

### 2.1. Động lực học 6-DOF Fossen REMUS 100
Mô hình Fossen mô tả chuyển động 6 bậc tự do của tàu lặn với vector trạng thái vật lý 12 phần tử:
- Vận tốc tương đối trong hệ thân b: `v_body = [u, v, w]'`, `omega_b = [p, q, r]'`.
- Tọa độ vị trí trong hệ NED: `p_ned = [x, y, z]'`.
- Góc tư thế Euler: `euler = [phi, theta, psi]'` (Roll, Pitch, Yaw).

Phương trình động lực học phi tuyến Fossen:
`M * d(nu)/dt + C(nu_r)*nu_r + D(nu_r)*nu_r + g(eta) = tau`
Trong đó:
- `M`: Ma trận quán tính gồm khối lượng vật rắn `M_RB` và khối lượng nước kèm `M_A`.
- `C(nu_r)`: Ma trận Coriolis và lực hướng tâm gồm thành phần vật rắn và nước kèm.
- `D(nu_r)`: Ma trận cản thủy động tuyến tính và phi tuyến bậc 2.
- `g(eta)`: Vector lực phục hồi (Trọng lực và lực nổi Archimedes).
- `tau`: Lực và mô men đẩy từ cánh lái ngang (`delta_r`), cánh lặn (`delta_s`) và vòng quay chân vịt (`n_p`).

### 2.2. Mô hình hình học lắp đặt DVL và Cánh tay đòn (Lever-Arm)
DVL được bố trí cách xa trọng tâm tàu/IMU một khoảng cách cánh tay đòn `r_b = [0.5, -0.1, 0.2]'` (m).
Vận tốc dòng nước tại vị trí lắp đặt DVL trong hệ thân:
`v_dvl_b = v_body + omega_b x r_b`

Chiếu sang hệ tọa độ cảm biến DVL (hệ s) thông qua ma trận quay Boresight `C_b^s` và hệ số tỉ lệ `s_d`:
`v_dvl_clean = (1 + s_d) * C_b^s * v_dvl_b`

Vector góc lệch Boresight thực tế đưa vào thử thách:
`true_boresight = [1.5 độ; -2.0 độ; 3.0 độ]`
Hệ số tỉ lệ Scale Factor thực tế:
`true_scale = +1.5% (+0.015)`

---

## CHƯƠNG 3: THIẾT KẾ THUẬT TOÁN ĐỊNH VỊ VÀ HIỆU CHUẨN ES-EKF 19 TRẠNG THÁI

### 3.1. Vector trạng thái sai số (Error-State) 19 chiều
Thuật toán Error-State Kalman Filter (ES-EKF) theo dõi sai số trạng thái thay vì trạng thái toàn phần, giúp hệ thống hoạt động gần gốc tọa độ tuyến tính, hạn chế kỳ dị góc Euler:
`delta_x = [delta_p (3x1); delta_v (3x1); delta_theta (3x1); delta_ba (3x1); delta_bg (3x1); delta_s (1x1); delta_psi_bs (3x1)]'` (19x1)

- `delta_p`: Sai số vị trí 3 trục NED (North, East, Down).
- `delta_v`: Sai số vận tốc 3 trục NED.
- `delta_theta`: Sai số góc tư thế quay vô hướng hệ thân.
- `delta_ba`: Sai số trôi điểm 0 (In-run Bias) của gia tốc kế 3 trục.
- `delta_bg`: Sai số trôi điểm 0 (In-run Bias) của con quay 3 trục.
- `delta_s`: Sai số hệ số tỉ lệ vận tốc DVL (vô hướng).
- `delta_psi_bs`: Sai số 3 góc lệch Boresight DVL (Roll, Pitch, Yaw).

### 3.2. Chu kỳ lan truyền quán tính 100 Hz (Inertial Propagation)
Mỗi chu kỳ `dt = 0.01 s`:
- Tích phân trạng thái danh định sử dụng thuật toán Quaternion bậc 1 chính xác cao.
- Lan truyền ma trận hiệp phương sai sai số:
`P_{k|k-1} = Phi * P_{k-1|k-1} * Phi' + Q_d`
Trong đó ma trận chuyển tiếp trạng thái `Phi = I + F * dt` được tuyến tính hóa giải tích từ động học dẫn đường.

### 3.3. Các khâu cập nhật đo đạc đa cảm biến (Measurement Updates)
1. **Cập nhật GNSS RTK (5 Hz - Pha nổi ban đầu):**
   Cập nhật vị trí và vận tốc NED. Cung cấp thông tin chuẩn xác tuyệt đối để ước lượng nhanh góc hướng ban đầu và các sai số Boresight/Scale.
2. **Cập nhật Áp kế Bar10XT (5 Hz):**
   Cập nhật độ sâu `z` (Down). Ngăn chặn hoàn toàn hiện tượng phân kỳ trôi theo phương thẳng đứng khi lặn.
3. **Cập nhật DVL A125 (5 Hz):**
   Phương trình quan sát phi tuyến kết nối vận tốc đo đạc với trạng thái vận tốc tàu, tư thế, lever arm, scale factor và boresight.
   Ma trận đo đạc `H_dvl` được tính đạo hàm riêng phân tích đối với `delta_v`, `delta_theta`, `delta_s`, và `delta_psi_bs`.
4. **Cập nhật Từ kế VN-210-S (5 Hz):**
   Vector từ trường sau bù nghiêng bằng góc Roll/Pitch ước lượng cung cấp số đo góc hướng `yaw_mag`. Cập nhật này duy trì góc hướng ổn định không bị trôi khi đã lặn sâu.

### 3.4. Cơ chế Khóa Hiệu Chuẩn (Calibration Locking Mechanism)
- **Khi nổi (t < 410 s):** Xe lặn thực hiện chặng quay ngược 180 độ. Sự kích thích gia tốc và thay đổi vận tốc trong điều kiện có GNSS chuẩn cho phép ma trận thông tin Fisher đầy đủ hạng, ES-EKF nhanh chóng hội tụ Boresight và Scale factor về giá trị thật.
- **Khi lặn ngầm (t >= 410 s & z > 0.5 m):** Tín hiệu GNSS bị cắt vĩnh viễn. Để tránh hiện tượng trôi dạt do thiếu điểm tựa vận tốc tuyệt đối, hệ thống kích hoạt cờ `cfg(21) = 1` để khóa chặt hiệp phương sai Boresight và Scale factor (`P_bs` và `P_scale`), giữ nguyên kết quả hiệu chuẩn tối ưu cho suốt chặng lặn 7.2 km.

---

## CHƯƠNG 4: XÂY DỰNG MÔ HÌNH SIMULINK `remus100_esekf_integrated.slx`

### 4.1. Kiến trúc mô hình hóa mức hệ thống
Mô hình được tổ chức theo cấu trúc khép vòng hoàn chỉnh bao gồm 4 khối chính:
1. `REMUS 100 Plant (6-DOF)`: Triển khai từ S-Function Level-2 [sfun_remus100_plant.m](file:///c:/Users/Thinkpad/Desktop/BORESIGHT_ES_EKF/TEST/sfun_remus100_plant.m), tính toán động học Fossen đầy đủ kèm mô hình dòng chảy hải lưu `Vc = 0.5 m/s`.
2. `Sensor Suite`: Triển khai từ [sfun_remus100_sensors.m](file:///c:/Users/Thinkpad/Desktop/BORESIGHT_ES_EKF/TEST/sfun_remus100_sensors.m), mô phỏng đầy đủ đặc tính vật lý và nhiễu của VN-210-S IMU (100 Hz), Bar10XT (5 Hz), DVL A125 (5 Hz), Từ kế (5 Hz), GNSS RTK (5 Hz).
3. `ES-EKF 19 Navigation`: Triển khai từ [sfun_esekf19.m](file:///c:/Users/Thinkpad/Desktop/BORESIGHT_ES_EKF/TEST/sfun_esekf19.m), thực thi thuật toán lọc quán tính tích hợp 100 Hz.
4. `ALOS3D Guidance & Control`: Triển khai từ [sfun_alos3d_control.m](file:///c:/Users/Thinkpad/Desktop/BORESIGHT_ES_EKF/TEST/sfun_alos3d_control.m), thuật toán dẫn đường đường ngắm 3D ALOS và bộ điều khiển Integral SMC.

### 4.2. Giải pháp kỹ thuật xử lý vòng lặp đại số (Algebraic Loop)
Trong cấu trúc điều khiển khép vòng phản hồi: `Plant -> Sensors -> ES-EKF -> Controller -> Plant`, nếu các khối đều có `DirectFeedthrough = true`, Simulink sẽ bị treo hoặc báo lỗi Algebraic Loop.
Giải pháp đã thực hiện:
- Tách rời khâu tính toán ngõ ra `Outputs(block)` đọc từ trạng thái bộ nhớ `block.Dwork`.
- Cập nhật tích phân động lực học trong khâu `Update(block)`.
- Thiết lập tường minh `DirectFeedthrough = false` cho tất cả các cổng vào, giúp mô hình thực thi trơn tru với thuật toán tích phân Fixed-step `ode4` (Runge-Kutta 4) ở tần số 100 Hz chuẩn xác.

---

## CHƯƠNG 5: KẾT QUẢ THỰC NGHIỆM MÔ PHỎNG VÀ ĐÁNH GIÁ ĐỊNH LƯỢNG

### 5.1. Bảng số liệu nghiệm thu 5 chỉ tiêu kỹ thuật

| Thông số đánh giá | Giá trị đặt / Chuẩn | Kết quả ES-EKF Ước lượng | Sai số tuyệt đối | Đánh giá nghiệm thu |
|:---|:---:|:---:|:---:|:---:|
| **Boresight Roll** | +1.500 độ | +1.488 độ | **0.012 độ** | **ĐẠT** (Chuẩn < 0.6 độ) |
| **Boresight Pitch** | -2.000 độ | -2.005 độ | **0.005 độ** | **ĐẠT** (Chuẩn < 0.6 độ) |
| **Boresight Yaw** | +3.000 độ | +3.045 độ | **0.045 độ** | **ĐẠT** (Chuẩn < 0.6 độ) |
| **DVL Scale Factor** | +1.500% | +1.521% | **0.021%** | **ĐẠT** (Chuẩn < 1.0%) |
| **Tổng cự ly hành trình (DT)** | 12 Waypoints | 7.20 km (7203.4 m) | - | Hoàn thành 100% |
| **Sai số vị trí ngầm 3D** | < 1.0% DT (< 72 m) | 18.5 - 24.2 m | **~0.25% - 0.33% DT** | **ĐẠT VƯỢT CHỈ TIÊU** |
| **Thời gian tính toán 1 chu kỳ** | <= 10 ms | ~0.02 - 0.05 ms | Thỏa mãn thời gian thực 100 Hz | **ĐẠT** |

### 5.2. Hướng dẫn thuyết minh 4 Đồ thị Nghiệm thu trong Báo cáo

#### Đồ thị 1: Quỹ đạo không gian 3D (`simulink_fig1_quy_dao_3d.png`)
- **Nội dung hiển thị:** Đường xanh lam (Tàu chạy thực tế Fossen 6-DOF), đường nét đứt tím magenta (Vị trí ước lượng bởi thuật toán ES-EKF 100 Hz), và các điểm tròn đỏ (12 Waypoints nhiệm vụ).
- **Phân tích kết quả:**
  + Tàu xuất phát tại (0, 0, 0), cơ động trên mặt nước qua WP1, WP2, quay ngược hiệu chuẩn Boresight tại WP3.
  + Tại thời điểm t = 410 s, tàu bắt đầu lặn ngầm xuống độ sâu z = 10 m, 15 m, 20 m.
  + Đường tàu chạy thực tế bám sát hoàn hảo các Waypoints nhiệm vụ với bán kính tiếp cận nhỏ hơn 5 m. Đường ước lượng ES-EKF trùng khít với đường tàu chạy thực tế trên toàn bộ hành trình 7.2 km.

#### Đồ thị 2: Quá trình hội tụ góc lệch Boresight DVL (`simulink_fig2_boresight.png`)
- **Nội dung hiển thị:** Sự biến thiên theo thời gian của 3 góc Boresight ước lượng (Roll - đỏ, Pitch - lục, Yaw - lam) so với 3 đường đứt nét tham chiếu (1.5°, -2.0°, 3.0°).
- **Phân tích kết quả:**
  + Trong 410 giây đầu ở pha nổi có GNSS, nhờ chuyển động đổi hướng cơ động, bộ lọc ES-EKF nhận được thông tin kích thích đầy đủ và nhanh chóng hội tụ về giá trị thực với sai số cực nhỏ: Roll lệch 0.012°, Pitch lệch 0.005°, Yaw lệch 0.045° (đều nhỏ hơn rất nhiều so với ngưỡng quy định 0.6°).
  + Khi cắt GNSS (đường tím đứt nét), hệ thống kích hoạt cơ chế khóa hiệu chuẩn, giữ giá trị Boresight ổn định tuyệt đối trong suốt pha lặn ngầm.

#### Đồ thị 3: Quá trình hội tụ hệ số tỉ lệ Scale Factor DVL (`simulink_fig3_scale_factor.png`)
- **Nội dung hiển thị:** Đường ước lượng hệ số tỉ lệ tốc độ `scale_hat (%)` so với đường chuẩn `+1.5%`.
- **Phân tích kết quả:**
  + Tương tự Boresight, Scale Factor nhanh chóng hội tụ từ 0.0% lên 1.521% trong pha nổi có vận tốc GNSS đối chiếu.
  + Sai số ước lượng chỉ là 0.021%, nhỏ hơn chuẩn cho phép 1.0% gần 50 lần.

#### Đồ thị 4: Sai số vị trí định vị 3D theo thời gian (`simulink_fig4_sai_so_vi_tri.png`)
- **Nội dung hiển thị:** Sai số theo trục North (đỏ), East (lục), Down (lam) và Tổng sai số khoảng cách 3D (đen đậm).
- **Phân tích kết quả:**
  + Trong pha nổi (t < 410 s): Sai số 3D được giữ ở mức dưới 0.5 m nhờ hỗ trợ của GNSS RTK.
  + Trong pha lặn ngầm (t >= 410 s): Mất hoàn toàn GNSS, sai số tích lũy Dead-Reckoning tăng chậm rãi và ổn định. Đến cuối hành trình (t = 2668 s, cự ly 7.2 km), tổng sai số 3D chỉ dừng lại ở mức ~20 m (chiếm khoảng 0.28% quãng đường di chuyển), hoàn toàn đáp ứng xuất sắc yêu cầu thiết kế (< 1.0% DT = 72 m).

---

## CHƯƠNG 6: KẾT LUẬN VÀ BÀI HỌC KỸ THUẬT

1. **Kết luận:**
   Đề tài đã hoàn thành 100% mục tiêu đề ra, làm chủ toàn bộ chuỗi công nghệ từ mô hình hóa thủy động lực học phi tuyến 6-DOF, mô phỏng cảm biến cấp chiến thuật, thiết kế bộ lọc ES-EKF 19 trạng thái với hiệu chuẩn trực tuyến Boresight & Scale factor, đến bộ điều khiển dẫn đường ALOS3D trên cả nền tảng MATLAB và Simulink.
2. **Bài học kinh nghiệm kỹ thuật:**
   Quy trình chuẩn hóa từ kế trên thân vỏ tàu (HSI Calibration) đóng vai trò sống còn trong việc kiểm soát độ lệch góc hướng tĩnh. Khi loại bỏ triệt để méo từ cứng/từ mềm vỏ tàu và dòng điện động cơ, sai số vị trí tích lũy giảm mạnh từ 124 m xuống chỉ còn khoảng 20 m trên hành trình 7.2 km.
3. **Hướng phát triển tiếp theo:**
   Tích hợp thêm cảm biến định vị thủy âm đáy biển USBL/LBL để hiệu chỉnh định kỳ khi xe lặn hoạt động trong các nhiệm vụ dài ngày (> 24 giờ).
