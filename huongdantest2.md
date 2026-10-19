Tôi đề xuất ba việc, theo đúng thứ tự:
Ưu tiên	Kiểm thử tiếp theo	Mục tiêu
1	Kiểm thử tích hợp đường đi dữ liệu thật	Đảm bảo test gọi đúng mô hình đo, Jacobian, R và reset covariance đang được esekf19_step.m sử dụng.
2	Monte Carlo Consistency Test, M = 1.000	Kiểm tra ANEES theo thời điểm, khoảng tin cậy và các nhóm trạng thái; dùng nhiều lần chạy độc lập, khởi tạo phù hợp với P0.
3	Calibration Convergence Test	Kiểm tra khả năng tự hiệu chuẩn từ scale = 0 và boresight = 0; đánh giá RMSE, trung vị và phân vị 95% của từng tham số tại các mốc thời gian.
Ở bước 2, với \(n_x=19\) và \(M=1000\), nếu mỗi lần chạy độc lập và mô hình thống kê phù hợp, khoảng tin cậy 95% cho ANEES trung bình tại một thời điểm cố định xấp xỉ [18,62; 19,38]. Không nên dùng tỷ lệ số điểm theo thời gian nằm trong khoảng này làm tiêu chí duy nhất.

Ở bước 3, cần chốt yêu cầu chính xác cần đạt cho từng góc boresight và scale factor trước khi đánh giá PASS/FAIL. Không nên tự chọn ngưỡng chỉ để các đường hội tụ trông đẹp.

4. Prompt tiếp theo để giao Claude
Hãy đưa prompt này cùng test2(2).m, esekf19_init(2).m, esekf19_step(2).m và hai script test hiện tại.
Hai bộ test test_esekf19_jacobian_reset.m và test_esekf19_dvl_leverarm.m hiện đều báo PASS.

Hãy tiếp tục kiểm thử bộ lọc ES-EKF 19 trạng thái, không lặp lại các test đơn vị đã hoàn thành và không tuning Q/P để làm đẹp chỉ số.

Nhiệm vụ 1 — Kiểm thử tích hợp production code

Rà soát xem các test hiện tại có thực sự kiểm tra mô hình đo, Jacobian, ma trận R và phép reset covariance đang được esekf19_step.m sử dụng hay chỉ kiểm tra công thức đã chép vào script test.

Nếu cần, tách hàm mô hình đo DVL và phép injection/reset thành hàm phụ trợ dùng chung cho cả production và test. Giữ tương thích với giao diện hiện tại của bộ lọc.

Thêm negative-control/mutation tests: cố tình đổi sai dấu tích có hướng, đổi dấu reset boresight hoặc bỏ thành phần nhiễu gyro khỏi R trong một bản test tạm thời. Chứng minh test tương ứng chuyển sang FAIL. Không giữ các lỗi cố ý trong bản production cuối cùng.

Nhiệm vụ 2 — Monte Carlo Consistency Test

Sử dụng chế độ CONSISTENCY và chạy 1.000 lần độc lập khi môi trường MATLAB cho phép. Kiểm tra các điều kiện khởi tạo và P0 trước khi chạy.

Thu thập ANEES theo thời gian và theo các nhóm trạng thái; tính khoảng tin cậy phù hợp với số lần chạy độc lập; báo riêng hai giai đoạn có GNSS và mất GNSS.

Đối với DVL, thu thập residual/innovation trước gating cùng covariance S tại từng lần đo. Tính NIS trước gating, tỷ lệ chấp nhận, trung bình innovation đã làm trắng, covariance của innovation đã làm trắng và tự tương quan theo từng thành phần qua nhiều độ trễ.

Không chỉ dựa vào NIS trung bình và tự tương quan của chuỗi NIS để kết luận innovation là trắng.

Nhiệm vụ 3 — Calibration Convergence Test

Đánh giá riêng chế độ CALIBRATION, trong đó ước lượng scale và boresight bắt đầu từ giá trị chưa hiệu chuẩn.

Thống kê sai số scale và ba góc boresight ở các mốc 5, 10, 15, 20 và 30 giây. Báo RMSE, trung vị, phân vị 95%, độ phân tán giữa các lần chạy và tỷ lệ hội tụ đạt yêu cầu.

Đặc biệt kiểm tra góc Roll vì log trước đó cho thấy sai số Roll lớn hơn Pitch và Yaw. Không đặt ngưỡng đạt yêu cầu cho đến khi nêu rõ căn cứ kỹ thuật hoặc yêu cầu định vị của dự án.

Định dạng đầu ra bắt buộc
1. Bảng ID | Test | Metric | Threshold | Actual | PASS/FAIL.
2. Kết quả theo từng thời điểm và từng nhóm trạng thái.
3. Biểu đồ innovation, NIS, ANEES, sai số scale và ba góc boresight, có đơn vị và đường giới hạn.
4. Bảng các lỗi đã xác minh, bằng chứng mã nguồn và cách tái hiện.
5. Kết luận rõ: đã xác minh điều gì, còn chưa xác minh điều gì và cần làm tiếp gì.
Chỉ ghi kết quả là PASS khi test thực sự thực thi và giá trị đo được đáp ứng ngưỡng. Nếu không chạy được MATLAB, phải ghi rõ chưa thực thi và không bịa kết quả.




Kết luận: giờ không cần thêm các phép kiểm tra đại số đơn giản nữa. Nên chuyển sang kiểm tra khả năng nhất quán qua Monte Carlo và khả năng hiệu chuẩn thực tế — đó mới là bước phân biệt một bộ lọc có các công thức đúng với một bộ lọc thực sự hoạt động tốt.