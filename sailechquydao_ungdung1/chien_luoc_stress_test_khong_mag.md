# CHIẾN LƯỢC NGHIÊN CỨU VÀ STRESS-TEST HỆ THỐNG ĐỊNH VỊ LIÊN HỢP ES-EKF 19 TRẠNG THÁI (CẤU HÌNH KHÔNG MAG)

---

## 1. BỐI CẢNH VÀ HIỆN TƯỢNG ĐÃ XÁC MINH

Trong thử nghiệm đối chiếu trực tiếp giữa mô phỏng chuẩn của GS. Thor I. Fossen và thuật toán ES-EKF 19 trạng thái trên tàu REMUS 100 (lộ trình 7 Waypoint 3D, thời gian 1.400 giây, quãng đường ~3.000 m):

1. **Bản Fossen Standalone (Lý tưởng)**: 
   - Tàu được cấp Ground Truth tuyệt đối, bám khít tim đường danh định.
   - Sai số dạt ngang Cross-track trung bình chỉ **0.41 m**, cực đại **7.32 m**.
   - Độ sâu bám chuẩn xác, sai số trung bình **0.13 m**.

2. **Bản kết hợp Fossen + ES-EKF (Thực tế cảm biến)**:
   - Tàu dẫn đường hoàn toàn dựa vào nghiệm ước lượng của ES-EKF trong điều kiện cảm biến có nhiễu, DVL bị lệch trục cơ khí thật (Roll = 1.50 deg, Pitch = -2.00 deg, Yaw = 3.00 deg, Scale factor = +1.50%).
   - ES-EKF khởi động nguội hoàn toàn (Cold Start: Boresight = [0; 0; 0], Scale = 0%).
   - Tàu chỉ có GNSS trong 30 giây đầu ở mặt nước, sau đó cắt hoàn toàn GNSS và lặn ngầm.
   - **Hiện tượng thực tế**: Tàu tự hiệu chuẩn DVL rất tốt (Pitch lệch 0.013 deg, Roll lệch 0.158 deg, Scale lệch 0.266%), nhưng **con tàu thật bị dạt ra xa khỏi tim đường danh định**:
     - Sai số dạt ngang Cross-track trung bình: **40.10 m**.
     - Sai số dạt ngang Cross-track cực đại: **590.80 m** (ở khúc ngoặt lặn sâu 100 m).
     - Sai số vị trí 3D cuối cùng: **176.76 m** sau 23.3 phút lặn ngầm (tỷ lệ trôi 5.97% quãng đường).

3. **Phát hiện cốt lõi về cảm biến La bàn từ (Magnetometer)**:
   - Thư mục [Sensor](file:///c:/Users/Thinkpad/Desktop/BORESIGHT_ES_EKF/Sensor) có sẵn tệp mã nguồn [Sensor/Mag.m](file:///c:/Users/Thinkpad/Desktop/BORESIGHT_ES_EKF/Sensor/Mag.m) mô phỏng từ kế 3 trục AHRS-10P.
   - Tuy nhiên, trong bộ lọc chính [algo/esekf19_step.m](file:///c:/Users/Thinkpad/Desktop/BORESIGHT_ES_EKF/algo/esekf19_step.m), chúng ta **chưa hề tích hợp kênh cập nhật La bàn Mag** (`measurement_flags` chỉ có GNSS, Depth, DVL; không có hàm `update_mag`).
   - Hậu quả: Khi lặn ngầm mất GNSS, góc hướng Yaw hoàn toàn bị "thả nổi", phụ thuộc thuần túy vào tích phân con quay FOG. Trôi góc hướng Yaw làm xoay xiên vector vận tốc DVL, gây ra độ dạt ngang hàng trăm mét trên mặt bằng North - East.

---

## 2. CHIẾN LƯỢC KỸ THUẬT: "STRESS-TEST CỰC HẠN KHÔNG MAG TRƯỚC"

Thay vì vội vàng tích hợp La bàn Mag vào để "chữa cháy", chiến lược nghiên cứu chuẩn mực được thống nhất như sau:

> **Nguyên tắc**: Tối ưu hóa và đẩy hệ thống đến giới hạn cực hạn trong cấu hình thuần **IMU + DVL + Depth (HOÀN TOÀN KHÔNG CÓ MAG)**. Khi quỹ đạo đã đạt mức tối ưu nhất có thể về mặt vật lý, ta mới tích hợp module Mag vào để đo lường chính xác bước nhảy vọt về hiệu năng.

### Lợi ích khoa học của chiến lược:
1. **Cô lập biến số (Isolation of Variables)**: Tránh để La bàn Mag "che giấu" các sai số tiềm ẩn trong mô hình động học INS, bù cánh tay đòn Lever-arm hay tốc độ hội tụ Boresight của DVL.
2. **Xác định trần hiệu năng lý thuyết (Performance Ceiling)**: Xác định rõ ràng hệ thống INS/DVL thuần túy có thể đạt độ chính xác tối đa là bao nhiêu khi không có hỗ trợ từ trường.
3. **Cung cấp bằng chứng đối chiếu đanh thép**: Khi Mag được thêm vào ở giai đoạn sau, ta sẽ có tập số liệu đối chứng khoa học tuyệt đối (A/B Testing minh bạch).

---

## 3. NGHIÊN CỨU TÀI LIỆU QUỐC TẾ: THỜI GIAN VÀ CƠ ĐỘNG MẶT NƯỚC CHUẨN

Tổng hợp từ các công trình khoa học uy tín (*IEEE Journal of Oceanic Engineering*, *Sensors*) và tài liệu vận hành thực tế của các dòng tàu ngầm tự hành (**REMUS 100**, **Kongsberg HUGIN**, **OceanServer Iver-3**, **WHOI AUV**):

### 3.1. Thời gian chạy pha hiệu chuẩn (Surface Calibration Phase)
- **Thời gian chuẩn thực địa**: Kéo dài từ **5 đến 15 phút** (phổ biến nhất là **5 đến 8 phút**).
  - Tàu REMUS 100 (Hydroid / WHOI): 5 - 8 phút chạy mặt nước để khóa GNSS RTK và căn chỉnh DVL/INS.
  - Tàu Iver-3 / Gavia: 5 - 10 phút chạy các đường cơ động cardinal (Bắc-Nam, Đông-Tây).
  - Tàu HUGIN (Kongsberg): 10 - 20 phút chạy căn chỉnh con quay và DVL trước các chuyến lặn sâu 3.000 m.

### 3.2. Hình dạng cơ động bắt buộc trên mặt nước (Không chạy đường thẳng)
Lý thuyết điều khiển (Kinsey & Whitcomb 2007) đã chứng minh: **Nếu tàu chạy thẳng, góc Roll Boresight sẽ không thể quan sát được (unobservable)**. Do đó, các kịch bản chuẩn luôn yêu cầu:
1. **Chạy hình số 8 (Figure-8 Maneuver) - 4 đến 6 phút**:
   - Tàu bẻ lái liên tục sang trái và phải, tạo góc nghiêng lắc Roll (3 đến 6 độ) và vận tốc dạt ngang Sway (v_y).
   - Đảo chiều dòng dạt để kích hoạt quan sát và triệt tiêu bias đối xứng cho cả **Roll Boresight** và **Yaw Boresight**.
2. **Tăng giảm tốc độ (Speed Variation) - 1 đến 2 phút**:
   - Tăng tốc và hãm tốc dọc trục X để kích thích kênh **Pitch Boresight** và **DVL Scale factor**.

### 3.3. Đánh giá lần chạy thử nghiệm vừa rồi
- Trong lần thử nghiệm vừa rồi, chúng ta chỉ cho GNSS chạy **30 giây** trên một đoạn đường thẳng tắp (~35 m).
- 30 giây là quá ngắn, tàu chưa có góc nghiêng Roll và dạt ngang Sway. Do đó khi cắt GNSS ở giây thứ 30, Boresight hoàn toàn chưa kịp hội tụ, dẫn đến sai số dạt tích lũy ban đầu lên tới 255 m.

---

## 4. LỘ TRÌNH THỰC HIỆN STRESS-TEST (CẤU HÌNH KHÔNG MAG)

Lộ trình gồm 3 giai đoạn chặt chẽ:

```
[ Giai đoạn 1: Chuẩn hóa pha mặt nước ]
  - Chạy cơ động mặt nước 3 - 5 phút (có GNSS, không Mag)
  - Khóa Boresight & Scale factor trước khi lặn
  - Lặn sâu 100m hoàn thành 7 Waypoints -> Đánh giá độ dạt giảm từ 590m xuống bao nhiêu
             │
             ▼
[ Giai đoạn 2: Stress-Test điều kiện cực hạn ]
  - Tăng dòng chảy đại dương: 0.5 m/s -> 1.0 m/s
  - Tăng sai số lắp đặt DVL: Boresight 5 độ, Scale factor 3%
  - Kiểm tra tính nhất quán NIS/ANEES và độ ổn định ma trận P
             │
             ▼
[ Giai đoạn 3: Xác lập trần hiệu năng & Mở kênh Mag ]
  - Đóng băng kết quả kịch bản Không Mag tốt nhất
  - Kích hoạt Sensor/Mag.m (bổ sung hàm update_mag vào ES-EKF)
  - So sánh trực tiếp: CÓ MAG vs KHÔNG CÓ MAG
```

---

## 5. BẢNG TIÊU CHÍ ĐÁNH GIÁ VÀ NGHIỆM THU (KPIS)

Mọi kịch bản thử nghiệm sẽ được đo lường tự động dựa trên bảng tiêu chí thống nhất:

| Chỉ số đánh giá | Lần chạy trước (30s cắt GNSS) | Mục tiêu Giai đoạn 1 (Chạy mặt nước 3-5 phút) | Mục tiêu Giai đoạn 3 (Khi bổ sung Mag) |
| :--- | :---: | :---: | :---: |
| **Boresight Pitch error** | 0.013 deg | < 0.050 deg | < 0.020 deg |
| **Boresight Roll error** | 0.158 deg | < 0.100 deg | < 0.050 deg |
| **Boresight Yaw error** | 0.501 deg | < 0.200 deg | < 0.050 deg |
| **Scale factor DVL error** | 0.266 % | < 0.100 % | < 0.050 % |
| **Cross-track error cực đại** | **590.80 m** | **< 15.00 m** | **< 5.00 m** |
| **Cross-track error trung bình**| **40.10 m** | **< 3.00 m** | **< 1.00 m** |
| **Sai số vị trí 3D cuối cùng** | **176.76 m** | **< 30.00 m** | **< 5.00 m** |
| **Tỷ lệ trôi theo quãng đường** | **5.97 %** | **< 1.00 %** | **< 0.20 %** |

---

*Tài liệu được thiết lập để theo dõi và chuẩn hóa toàn bộ quá trình thực nghiệm đối chứng trong thư mục `sailechquydao_ungdung1/`.*
 

  // tàu remus 100 setting up chạy luôn 1.5m/s ma trận Po  dự báo độ bất định setup 0 => nis lên cao => từ chối số liệu DVL và sensor