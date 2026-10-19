sau khi đọc cả test2.m, esekf19_init.m và esekf19_step.m, tôi muốn điều chỉnh thứ tự đề xuất một chút: chưa nên bắt đầu bằng việc thêm \(\Delta R_{\text{gyro}}\) ngay. Trước tiên cần kiểm tra Jacobian và quy ước sai số, sau đó mới kiểm tra nhiễu DVL bằng Monte Carlo.
Lý do là trong code có ít nhất ba nguồn sai lệch cần phân biệt: mô hình đo DVL, phép reset covariance của boresight và sai khác giữa mô hình động học rời rạc với mô hình sinh dữ liệu. Nếu sửa nhiều thứ cùng lúc, kết quả đẹp lên cũng không cho biết nguyên nhân thực sự.
Kế hoạch thực hiện đề xuất
1

Đóng băng kết quả gốc và chuẩn bị dữ liệu đối chứng
Bắt đầu ở đây
Giữ nguyên seed rng(1,'twister'), các thông số hiện tại, M_runs = 200 và toàn bộ ba file. Lưu riêng kết quả gốc; ngoài các chỉ số tổng hợp, nên ghi lại NIS theo từng phép đo, trạng thái chấp nhận/từ chối và ANEES tại các mốc 1, 5, 10, 15, 20, 25, 30 giây.
Tại sao? Khi chỉ thay đổi một công thức và giữ nguyên các số ngẫu nhiên, ta có thể so sánh theo từng kịch bản thay vì phải đoán xem khác biệt đến từ ngẫu nhiên Monte Carlo hay từ thay đổi thuật toán.
Một chỉ số nên theo dõi đầu tiên là ANEES hiệu chuẩn DVL, hiện bằng 5.18 so với kỳ vọng 4.00; bên cạnh đó là NIS DVL 3.62 so với kỳ vọng 3.00.



2

Kiểm tra Jacobian DVL và phép reset boresight trước khi chạy Monte Carlo
Đây là bước tôi đánh giá cao hơn việc chỉnh \(R\) ngay từ đầu.
Trong esekf19_step.m, ma trận \(H_{\text{DVL}}\) được xây dựng thủ công cho vận tốc, tư thế, gyro bias, scale và boresight. Sau khi cập nhật, inject_error cũng reset covariance thông qua ma trận \(G\). esekf19_step.m
 esekf19_step.m

Tôi đề xuất viết các bài kiểm tra số nhỏ, không cần chạy 200 kịch bản:
- Kiểm tra Jacobian: thay đổi lần lượt vận tốc, góc tư thế, gyro bias, scale và boresight một lượng rất nhỏ. So sánh biến thiên của hàm dự đoán DVL với \(H\,\delta x\), dùng sai phân hữu hạn trung tâm.
- Kiểm tra phép reset: kiểm tra xem sai số boresight sau khi inject có biến đổi theo đúng Jacobian reset \(G\) hay không.
- Kiểm tra GNSS yaw: làm tương tự cho hàng Jacobian yaw tại một số tư thế đại diện, đặc biệt để phát hiện sai dấu hoặc nhầm quy ước khung tọa độ.
Có một điểm đáng ngờ cụ thể: code dùng \(G_{\theta}=I-\frac12[\delta\theta\times]\) cho tư thế, nhưng cũng dùng dấu trừ tương tự cho boresight, dù phép cập nhật boresight được định nghĩa bằng \(C_{bs,\text{new}}=C_{bs,\text{old}}\exp(-[\delta b_s\times])\). Với quy ước này, khối reset boresight có khả năng cần dấu ngược lại. Đây là điều cần xác minh bằng Jacobian số, không nên sửa dấu một cách mù quáng.
Tại sao làm bước này trước? Nếu \(H\) hoặc \(G\) sai, phép cập nhật có thể tạo covariance sai ngay cả khi \(R\) mô tả nhiễu cảm biến hoàn hảo. Điều chỉnh \(R\) khi đó chỉ có nguy cơ che lấp lỗi.



3

Kiểm tra mô hình DVL trong điều kiện không nhiễu
Hiện tại, bộ sinh dữ liệu tạo vận tốc tại DVL bằng vận tốc thật và vận tốc góc thật ở thời điểm cập nhật; trong khi bộ lọc dùng gyro_meas - gyro_bias để tính bù lever arm. test2.m
 esekf19_step.m

Tạo một bài kiểm tra riêng trong đó trạng thái, tư thế, vận tốc góc và các tham số hiệu chuẩn đều đã biết chính xác, không có nhiễu. Tính phép đo DVL dự báo và residual.
Ở phép thử đơn giản nhất, residual phải tiến về 0 khi hai phía sử dụng cùng quy ước tọa độ và cùng thời điểm. Nếu không, cần tìm nguyên nhân trong dấu của lever arm, thứ tự nhân ma trận quay, định nghĩa boresight hoặc lệch thời điểm; chưa nên bàn tới covariance nhiễu.
Sau đó kiểm tra riêng sai số do khác cách tích phân. test2.m dùng vận tốc trung điểm và tích phân vị trí hình thang, còn esekf19_step.m dùng tư thế ở đầu bước và cập nhật vị trí bằng vận tốc mới. test2.m
 esekf19_step.m

Tại sao? Một hệ thống có thể tạo residual không bằng 0 ngay cả khi không có nhiễu. Khi đó, thêm nhiễu vào \(R\) không sửa được nguyên nhân gốc; nó chỉ làm bộ lọc bớt tin vào phép đo.



4

E1 — Chỉ kiểm tra ảnh hưởng của nhiễu gyro qua lever arm
Sau khi các kiểm tra ở bước 2–3 đạt yêu cầu, mới thực hiện phép thử này.
Mô hình bù lever arm của bộ lọc dùng tốc độ góc lấy từ mẫu gyro có nhiễu trắng. Với \(\sigma_g=0.02\ \mathrm{rad/s}\), \(r=[0.5,-0.1,0.2]^T\ \mathrm m\) và scale gần giá trị thật, thành phần covariance bổ sung có độ lớn trace xấp xỉ \(2.47\times10^{-4}\ \mathrm{m^2/s^2}\). Giá trị này tương đối lớn so với trace \(R_{\text{DVL}}\) hiện tại khoảng \(1.02\times10^{-3}\ \mathrm{m^2/s^2}\).
Chạy hai phiên bản với cùng seed:
E1-A: code gốc.
E1-B: bổ sung thành phần covariance do nhiễu gyro truyền qua lever arm vào mô hình đo DVL; không đổi các phần khác.
So sánh NIS DVL, tỷ lệ reject, ANEES calibration, velocity, position và ANEES tổng thể. Khi triển khai, phải dùng ma trận covariance đầy đủ theo công thức, không chỉ cộng một hằng số vào đường chéo.
Tại sao? Đây là thử nghiệm trực tiếp nhất để xem thiếu thành phần nhiễu nói trên có góp phần giải thích NIS DVL cao hay không.
Tuy nhiên, có một giới hạn quan trọng: cùng mẫu gyro_meas này đã được dùng trong bước truyền trạng thái INS và sau đó lại dùng trong dự báo DVL. Vì thế, nhiễu giữa trạng thái tiên nghiệm và phép đo DVL có thể tương quan. Chỉ cộng \(\Delta R_{\text{gyro}}\) là phép thử chẩn đoán hợp lý, chưa chắc là mô hình cuối cùng chính xác.



5

E2 — Phân biệt nhiễu gyro với sai lệch mô hình và tương quan nhiễu
Nếu E1 cải thiện kết quả, hãy thực hiện một phép đối chứng thứ hai thay vì chốt luôn công thức bổ sung \(R\).
Trước hết, trong mô phỏng chẩn đoán, thay riêng tốc độ góc dùng cho bù lever arm bằng tốc độ góc thật đã biết. Đây là phép thử dùng thông tin oracle, chỉ dùng để chẩn đoán chứ không phải giải pháp triển khai. Các phần còn lại giữ nguyên.
Nếu kết quả thay đổi rõ so với code gốc, điều này cho thấy khâu sử dụng tốc độ góc trong mô hình DVL có ảnh hưởng đáng kể. Sau đó mới xác định cách xử lý thực tế: mô hình hóa covariance và tương quan nhiễu đúng hơn, hoặc cải thiện cách ước lượng tốc độ góc dùng trong bù lever arm.
Nếu E1 không cải thiện, chưa thể kết luận giả thuyết nhiễu gyro sai hoàn toàn; nguyên nhân có thể nằm ở sai dấu Jacobian, sai lệch động học hoặc tương quan nhiễu chưa được xử lý.
Tại sao? Một phép thử oracle cho biết giới hạn cải thiện khi loại bỏ sai số tốc độ góc trong khâu bù lever arm. Nó giúp phân biệt “nhiễu đo chưa được đưa vào \(R\)” với “mô hình dự báo DVL hoặc covariance toàn hệ thống chưa đúng”.



6

Kiểm tra tính nhất quán của \(Q\) và phép lan truyền covariance
esekf19_init.m xây dựng khối \(Q_{pp},Q_{pv},Q_{vv}\) từ mô hình gia tốc trắng liên tục, còn esekf19_step.m lan truyền covariance bằng ma trận chuyển tiếp rời rạc bậc một. esekf19_init.m
 esekf19_step.m

Tôi chưa coi đây là lỗi chắc chắn; nó là một xấp xỉ cần xác minh. Hãy kiểm tra bằng mô phỏng đơn giản với IMU noise đã biết, so sánh covariance dự báo với phân bố sai số thực và kiểm tra độ nhạy khi giảm dt.
Nếu sai số hội tụ theo dt, có thể đang thấy ảnh hưởng rời rạc hóa. Nếu vẫn còn sai lệch có cấu trúc, cần xem lại mô hình trạng thái sai số, các khối coupling trong \(F\), và cách xây dựng \(Q\) từ cùng mô hình liên tục.
Tại sao? Việc thay \(Q_{pp}\) hay \(Q_{pv}\) ngay bây giờ chỉ dựa trên ANEES tổng thể là chưa có cơ sở đủ chặt chẽ.



7

Cuối cùng mới xử lý gate-lock của lần chạy 815
Chỉ thực hiện sau khi các kiểm tra trước đã hoàn thành.
Tái tạo seed của lần 815, lưu innovation, \(S\), NIS từng cảm biến, covariance và sai số thật theo từng bước. Xác minh chính xác chuỗi chấp nhận/từ chối thay vì chỉ nhìn RMSE cuối cùng.
Không nên chữa bằng cách nới ngưỡng Chi-square cho mọi phép đo hoặc luôn ép nhận GNSS/DVL khi bị từ chối. Cách đó có thể chữa một lần chạy nhưng làm sai thống kê cho hàng loạt lần chạy bình thường.
Nếu xác nhận được một kịch bản khởi tạo lỗi lớn và các phép đo độc lập vẫn hợp lệ, có thể thiết kế một chế độ phục hồi riêng: kiểm tra chuỗi innovation, tái khởi tạo hoặc tăng bất định cho những trạng thái thực sự mất tin cậy, rồi quay lại chế độ lọc bình thường khi có bằng chứng đủ mạnh. Phải kiểm tra false acceptance và NIS/ANEES trên cả mẫu bình thường lẫn mẫu khó.
Tại sao để sau cùng? Một cơ chế phục hồi sẽ khó đánh giá nếu mô hình đo hoặc covariance vốn đã sai. Trước hết phải chắc rằng bộ lọc bình thường đang tính đúng.



6. Có một vấn đề trong cách đánh giá consistency cần chỉnh
Trong test2.m, code tính khoảng tin cậy ANEES theo \(M n_x\), sau đó đếm tỷ lệ thời điểm nằm trong khoảng và kết luận đạt khi tỷ lệ đó từ 80% trở lên. test2.m

Cần giữ khoảng tin cậy 95% tại từng thời điểm, nhưng không nên dùng riêng ngưỡng 80% này như tiêu chuẩn thống kê chính thức. Các thời điểm liên tiếp có tương quan, và ngưỡng 80% chưa được biện minh bằng một kiểm định cụ thể.
Tôi đề xuất báo cáo thêm ANEES tại các mốc được định trước, cùng khoảng tin cậy 95% cho từng mốc và thống kê theo hai pha có/mất GNSS. Việc này không làm thay đổi bộ lọc, chỉ làm cho kết luận consistency đáng tin cậy hơn.
Dù vậy, ANEES tổng thể 21.31, cao hơn 19 khoảng 12.2%, cùng ANEES hiệu chuẩn DVL 5.18/4 vẫn là tín hiệu đủ mạnh để tiếp tục điều tra. Không cần chờ sửa biểu đồ mới bắt đầu kiểm tra mô hình.
Quyết định tôi khuyên bạn đưa ra ngay lúc này
Tôi sẽ làm bước 2 trước, rồi bước 3, sau đó mới E1 và E2. Đó là thứ tự giảm nguy cơ tốn hàng trăm lượt Monte Carlo mà vẫn không biết lỗi nằm ở đâu.
Bước cần triển khai đầu tiên không phải chỉnh \(Q/R\), mà là kiểm tra bằng sai phân hữu hạn xem các Jacobian DVL và dấu của khối reset boresight có đúng với chính quy ước \(C_{bs}=\exp(-[bs\times])\) của code không. Nếu sai, sửa riêng lỗi đó và chạy lại seed đối chứng trước khi bổ sung bất kỳ thành phần covariance mới nào.