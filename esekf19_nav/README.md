# Gói ROS 2 Dẫn Đường ES-EKF 19 Trạng Thái (`esekf19_nav`)

Node ROS 2 thực hiện thuật toán **Bộ lọc Kalman mở rộng trạng thái sai số (19-State ES-EKF)** kết hợp ước lượng và hiệu chuẩn trực tuyến góc lệch lắp đặt cơ khí (**Boresight**) và hệ số tỉ lệ co giãn âm học (**Scale factor**) của DVL, phục vụ dẫn đường chính xác cao cho tàu lặn tự hành (AUV/UUV) trên mô phỏng Gazebo.

---

## 1. Yêu Cầu Môi Trường & Phiên Bản Đã Thử Nghiệm
* **Hệ điều hành:** Ubuntu 22.04 LTS / 24.04 LTS (hoặc WSL2 / Docker trên Windows).
* **Phiên bản ROS 2:** 
  * ROS 2 **Humble Hawksbill** (Khuyến nghị chuẩn LTS).
  * ROS 2 **Iron Irwini** / **Jazzy Jalisco** (Tương thích hoàn toàn).
* **Thư viện phụ thuộc:** `Python 3.10+`, `numpy >= 1.20`.

---

## 2. Hướng Dẫn Biên Dịch & Cài Đặt (Build)

Sao chép thư mục `esekf19_nav` vào `src` của ROS 2 workspace:

```bash
# 1. Tạo thư mục workspace nếu chưa có
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Copy thư mục esekf19_nav vào ~/ros2_ws/src/
# 3. Cài đặt các gói phụ thuộc ROS 2
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Biên dịch gói bằng colcon
colcon build --packages-select esekf19_nav --symlink-install

# 5. Nạp môi trường
source install/setup.bash
```

---

## 3. Hướng Dẫn Chạy Node (Run / Launch)

### Cách 1: Khởi động kèm file Launch (Khuyến nghị)
```bash
ros2 launch esekf19_nav demo.launch.py
```
*(Có thể đổi file tham số ngoài: `ros2 launch esekf19_nav demo.launch.py params_file:=/duong/dan/params_custom.yaml`)*

### Cách 2: Chạy trực tiếp Node với tham số YAML
```bash
ros2 run esekf19_nav esekf19_node --ros-args --params-file src/esekf19_nav/config/params.yaml
```

---

## 4. Bảng Giao Tiếp Topic ROS 2

### Topic Đầu Vào (Subscriptions - QoS `sensor_data`)
| Tên Topic Mặc Định | Kiểu Message ROS 2 | Tần Số | Mô Tả |
| :--- | :--- | :--- | :--- |
| `/imu/data` | `sensor_msgs/msg/Imu` | 100 - 400 Hz | Gia tốc tuyến tính và vận tốc góc khung thân (được dùng để chạy tầng dự báo INS). |
| `/dvl/velocity` | `geometry_msgs/msg/TwistWithCovarianceStamped` | 5 - 10 Hz | Vận tốc đáy 3 trục (vx, vy, vz) tại vị trí DVL. |
| `/depth/pressure` | `sensor_msgs/msg/FluidPressure` | 5 - 10 Hz | Áp suất thủy tĩnh nước biển (Pascals), tự động quy đổi thành độ sâu z. |
| `/gnss/odometry` | `nav_msgs/msg/Odometry` | 1 Hz | Vị trí NED, vận tốc NED và hướng yaw từ GNSS RTK 2 ăng-ten khi ở mặt nước. |

### Topic Đầu Ra (Publishers)
| Tên Topic Mặc Định | Kiểu Message ROS 2 | Tần Số | Mô Tả |
| :--- | :--- | :--- | :--- |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | Theo IMU (100-400Hz) | Vị trí NED (x,y,z), quaternion tư thế, vận tốc và ma trận hiệp phương sai 6x6. |
| `/esekf/calibration` | `std_msgs/msg/Float64MultiArray` | Theo DVL (5-10Hz) | Tham số hiệu chuẩn trực tuyến: `[Roll_bs(°), Pitch_bs(°), Yaw_bs(°), Scale(%), ba_xyz(mg), bg_xyz(°/h)]`. |

---

## 5. Bảng Tham Số ROS Parameter (Cấu hình trong `config/params.yaml`)

| Nhóm | Tên Tham Số | Mặc Định | Đơn Vị | Ý Nghĩa Kỹ Thuật |
| :--- | :--- | :--- | :--- | :--- |
| **system** | `lever_arm` | `[0.5, -0.1, 0.2]` | m | Cánh tay đòn [x, y, z] từ IMU đến DVL trong khung thân. |
| **system** | `latitude_deg` | `20.8` | deg | Vĩ độ địa phương để tính vận tốc góc tự quay Trái Đất. |
| **system** | `water_density` | `1025.0` | kg/m³ | Khối lượng riêng nước biển để chuyển áp suất thành độ sâu. |
| **noise** | `gyro_arw` | `3.49e-6` | rad/√s | Nhiễu góc con quay FOG (0.012°/√h). |
| **noise** | `acc_vrw` | `0.002` | m/s/√s | Nhiễu vận tốc gia tốc kế. |
| **noise** | `dvl_noise_floor` | `0.01` | m/s | Nhiễu sàn DVL. |
| **operation** | `lock_calibration_underwater`| `true` | bool | **Khóa cứng** Boresight & Scale khi lặn ngầm (>5s không GNSS). |
| **operation** | `max_dt_threshold` | `0.5` | s | Ngưỡng an toàn bảo vệ bộ lọc khi bị gián đoạn/nhảy bước thời gian. |

---

## 6. Giả Định Và Những Điều Cần Kiểm Tra Với Partner (Gazebo Checklist)

> [!IMPORTANT]
> Khi partner đưa gói mô phỏng Gazebo lên tích hợp, cần kiểm tra 4 điểm cốt lõi sau:

1. **Khớp kiểu Message của DVL trong Gazebo:**
   * Node hiện tại mặc định lắng nghe `geometry_msgs/msg/TwistWithCovarianceStamped`.
   * *Nếu Gazebo dùng plugin DVL phát ra `uuv_sensor_ros_plugins_msgs/msg/DVL` hoặc `acoustic_msgs/msg/Dvl`:* Hãy mở file `esekf19_nav/esekf19_node.py` tại hàm `dvl_callback` và ánh xạ lại 3 trường vận tốc: `linear.x`, `linear.y`, `linear.z`.
2. **Khớp kiểu Message của Cảm biến Độ sâu:**
   * Node hiện tại mặc định nhận áp suất `sensor_msgs/msg/FluidPressure` (Pascals) và tự động tính `depth = (P - 101325)/(rho * g)`.
   * *Nếu Gazebo phát trực tiếp độ sâu z (ví dụ `geometry_msgs/msg/PoseWithCovarianceStamped` hoặc `std_msgs/msg/Float64`):* Hãy cập nhật trong `depth_callback`.
3. **Quy ước Hệ tọa độ (NED vs ENU / NWU):**
   * Thuật toán ES-EKF dùng chuẩn hàng hải quốc tế **NED (North - East - Down)**.
   * Gazebo thông thường dùng **ENU (East - North - Up)** hoặc **NWU**. Hãy đảm bảo plugin IMU và GNSS trong Gazebo đã xuất đúng hướng NED hoặc cấu hình TF transform phù hợp.
4. **Vị trí Cánh tay đòn `lever_arm`:**
   * Cần kiểm tra file URDF/SDF của tàu trong Gazebo để đo chính xác khoảng cách từ gốc link IMU đến link DVL, sau đó điền vào `system.lever_arm` trong `config/params.yaml`.
