Tôi đang phát triển bộ lọc ES-EKF 19 trạng thái cho định vị UUV. Hãy đọc trực tiếp và phân tích kỹ ba file đính kèm:

- `test2.m`
- `esekf19_init.m`
- `esekf19_step.m`

Nhiệm vụ hiện tại chỉ tập trung vào hai nhóm kiểm thử:

1. Kiểm tra Jacobian và reset covariance.
2. Kiểm tra mô hình đo DVL, nhiễu gyro và cánh tay đòn IMU–DVL.

Không thực hiện tuning Q/P. Không thay đổi công thức của bộ lọc chỉ để làm cho chỉ số đẹp hơn. Trước hết phải xây dựng test để phát hiện lỗi và cung cấp bằng chứng.

## A. Rà soát mã nguồn trước khi viết test

Hãy lần theo các đoạn mã thực tế:

- `test2.m`: cách sinh quỹ đạo, mẫu gyro, phép đo DVL, cấu hình nhiễu và thời điểm cập nhật.
- `esekf19_step.m`: hàm dự đoán DVL, Jacobian H, ma trận R, innovation và các hàm update.
- `inject_error`: phép injection quaternion, cập nhật boresight trên SO(3) và ma trận reset covariance G.
- `esekf19_init.m`: định nghĩa Q, P0 và các tham số nhiễu.

Trước khi viết code test, lập bảng gồm vị trí trong mã, phương trình hiện tại, giả thiết toán học và vấn đề cần kiểm tra.

Chú ý kiểm tra cấu hình thật trong file. Phiên bản đang được đưa vào phân tích có `test_mode = 'CALIBRATION'` và `M_runs = 100`, trong khi một số chú thích cũ ghi 200. Hãy báo đúng cấu hình thực thi, không tự đồng nhất nó với log của các phiên bản trước.

## B. Bộ test 1: Jacobian và reset covariance

### B1. Kiểm tra Jacobian đo DVL bằng sai phân hữu hạn

Dùng chính hàm mô hình đo đang được bộ lọc thực thi. Không viết một bản sao của công thức trong test rồi so sánh hai bản sao với nhau.

Nếu cần, được phép tách mô hình đo DVL thành hàm phụ trợ độc lập hoặc bổ sung đầu ra chẩn đoán tùy chọn, nhưng phải giữ nguyên API cũ và không thay đổi công thức đang dùng trước khi kiểm thử.

So sánh Jacobian giải tích `H_dvl` với Jacobian sai phân hữu hạn trung tâm:

`Hfd(:,j) = (h(x ⊕ eps_j e_j) - h(x ⊕ (-eps_j e_j))) / (2*eps_j)`

Phép nhiễu trạng thái phải dùng đúng quy ước ES-EKF:

- Quaternion được nhiễu bằng phép nhân quaternion phù hợp với `inject_error`.
- Boresight được nhiễu bằng phép nhân ma trận quay trên SO(3), phù hợp với `Cbs_new = Cbs_old * exp(-[delta_bs]x)`.
- Các trạng thái tuyến tính dùng phép cộng phù hợp.
- Giữ nguyên mẫu gyro đo khi tính đạo hàm theo trạng thái.

Kiểm tra tối thiểu 50 trạng thái khác nhau, có thay đổi vận tốc, tư thế, gyro bias, scale, boresight và cánh tay đòn khác không. Dùng sai phân trung tâm và ít nhất hai cỡ bước sai phân để kiểm tra độ ổn định số.

Các chỉ tiêu:

- Sai số tương đối lớn nhất của từng cột Jacobian có ảnh hưởng đáng kể không vượt `1e-5`.
- Các cột tương ứng vị trí và bias gia tốc kế, vốn không xuất hiện trực tiếp trong mô hình đo DVL hiện tại, phải có sai số tuyệt đối không vượt `1e-8`.
- Xuất sai số tuyệt đối, sai số tương đối và chỉ số cột lớn nhất.
- Báo riêng mọi thành phần đạo hàm bị bỏ qua, kể cả khi chúng rất nhỏ.

Nếu phép thử thất bại, hãy chỉ ra đúng khối H, chiều sai lệch, khả năng sai dấu hoặc đạo hàm còn thiếu. Chưa sửa thuật toán trước khi có kết quả.

### B2. Kiểm tra reset Jacobian cho tư thế

Kiểm tra ma trận reset hiện dùng:

`G_theta = I - 0.5 * skew(delta_theta)`

Hãy tự xây dựng phép ánh xạ sai số sau injection trên SO(3), rồi tính Jacobian bằng sai phân hữu hạn độc lập. Không giả định dấu hiện tại đúng hoặc sai chỉ từ comment trong code.

Thử correction có độ lớn khoảng `1e-6`, `1e-4`, `1e-3` và `1e-2` rad.

Điều kiện đề xuất:

- Với `norm(delta_theta) <= 1e-3`, sai khác tuyệt đối lớn nhất giữa Jacobian reset số và công thức bậc nhất không quá `1e-6`.
- Với `norm(delta_theta) <= 1e-2`, sai khác tuyệt đối lớn nhất không quá `1e-4`.
- Nếu không đạt, xác định đó là lỗi dấu/công thức hay sai khác do xấp xỉ bậc nhất.

### B3. Kiểm tra reset Jacobian cho boresight

Kiểm tra riêng:

`G_bs = I + 0.5 * skew(delta_bs)`

Dựa vào quy ước `Cbs = exp(-[boresight]x)` và phép injection thực tế. Dẫn xuất hàm ánh xạ sai số trước–sau reset rồi đối chiếu Jacobian số.

Dùng các mức correction như B2. Nếu dấu trong code sai, phải chứng minh bằng Jacobian hữu hạn và phương trình, không kết luận dựa vào trực giác.

### B4. Kiểm tra tính hợp lệ của covariance

Với covariance dương xác định đầu vào và các correction đại diện, chạy qua phép reset thực tế, sau đó kiểm tra:

- `P` có kích thước 19×19.
- Sai số đối xứng tương đối `norm(P-P','fro')/max(norm(P,'fro'),eps)` không quá `1e-12`.
- Trị riêng nhỏ nhất không âm đáng kể; sử dụng ngưỡng sai số số học được nêu rõ.
- Kết quả covariance có khớp với phép tính `G * P_post * G'` được triển khai hay không.

Hãy test cả tư thế lẫn boresight, không chỉ kiểm tra covariance sau các lần chạy Monte Carlo.

## C. Bộ test 2: DVL, gyro và cánh tay đòn

### C1. Kiểm tra dấu tích có hướng và bất biến vật lý

Dùng ví dụ:

`omega = [0.2; -0.3; 0.4] rad/s`

`r = [0.5; -0.1; 0.2] m`

Kết quả kỳ vọng:

`cross(omega,r) = [-0.02; 0.16; 0.13] m/s`

Sai lệch cho phép không quá `1e-12 m/s`.

Sau đó kiểm tra hai điều kiện:

1. Khi `r = [0;0;0]`, thay đổi gyro không được làm thay đổi vận tốc DVL dự đoán.
2. Khi vận tốc góc tương đối bằng 0, mô hình phải trở về `(1+s)*Cbs*v_body`.

Sai lệch số học của các phép thử bất biến không quá `1e-12 m/s`.

### C2. Kiểm tra tính nhất quán của hai mô hình đo

Đối chiếu trực tiếp phần sinh DVL trong `test2.m` và phần dự đoán DVL trong `esekf19_step.m`.

Mô hình sinh dữ liệu đang dùng vận tốc thân và vận tốc góc thật ở `t_target`. Gyro đưa vào bộ lọc được tạo tại `t_mid`. Trong cập nhật DVL, bộ lọc sử dụng mẫu gyro đó để tính vận tốc tại đầu đo.

Viết một test hai phần:

**Phần 1: Mô hình lý tưởng cùng timestamp**

Cho bộ sinh và bộ lọc dùng cùng trạng thái thật, cùng thời điểm, không nhiễu và cùng các quy ước tọa độ. Residual DVL phải gần 0, với sai số lớn nhất không quá `1e-10 m/s`.

**Phần 2: Kiểm tra pipeline hiện tại**

Giữ nguyên quy ước thời điểm đang sử dụng. Đo sai lệch vận tốc chỉ do khác timestamp giữa gyro và DVL, gồm:

- RMS độ lớn sai lệch vận tốc.
- Giá trị lớn nhất.
- Sai lệch theo từng trục.
- Tỷ số RMS sai lệch trên độ lệch chuẩn DVL hiệu dụng.
- Thời điểm xuất hiện sai lệch lớn nhất.

Ngưỡng chấp nhận kỹ thuật ban đầu cho phần 2 là RMS không quá 10% độ lệch chuẩn hiệu dụng và sai lệch lớn nhất không quá 25%. Ghi rõ đây là ngưỡng đánh giá của dự án, không phải tiêu chuẩn phổ quát. Nếu vượt ngưỡng, đề xuất căn chỉnh timestamp hoặc nội suy vận tốc góc phù hợp trước khi chỉnh Q/P.

### C3. Kiểm tra phương sai do nhiễu gyro qua lever arm

Với cấu hình hiện tại:

- `dt = 0.0025 s`.
- `sigma_arw = 0.001 rad/sqrt(s)`.
- `lever_arm = [0.5; -0.1; 0.2] m`.
- `true_scale = 0.015`.

Xác minh cách quy đổi:

`sigma_g = sigma_arw / sqrt(dt) = 0.02 rad/s`

Đối chiếu công thức R trong code với Jacobian truyền nhiễu gyro qua mô hình đo:

`R_gyro = (1+s)^2 * Cbs * skew(r) * Sigma_g * skew(r)' * Cbs'`

Trong đó `Sigma_g = sigma_g^2 * eye(3)`.

Sử dụng ít nhất 100.000 mẫu nhiễu độc lập để đánh giá:

- Trung bình sai số do gyro.
- Hiệp phương sai thực nghiệm của thành phần lever-arm.
- Hiệp phương sai lý thuyết của thành phần lever-arm.
- Hiệp phương sai tổng khi có thêm nhiễu DVL nội tại.
- Sai khác giữa từng phần tử hiệp phương sai thực nghiệm và lý thuyết.

Với `N` mẫu, dùng thước đo:

`z_ij = (R_emp(i,j)-R_theory(i,j)) / sqrt((R_theory(i,i)*R_theory(j,j)+R_theory(i,j)^2)/(N-1))`

Điều kiện PASS đề xuất là sai lệch chuẩn hóa của trung bình và hiệp phương sai không quá 4. Ghi rõ số mẫu và giá trị thực tế.

Không tự viết lại riêng công thức R trong test rồi tuyên bố production code đã đúng. Hãy lấy ma trận R từ chính hàm sản xuất hoặc từ giao diện chẩn đoán của hàm đó.

### C4. Kiểm thử nhiễu trong quá trình tích hợp INS–DVL

Cần đánh giá một giới hạn quan trọng: cùng mẫu gyro tham gia cả quá trình lan truyền trạng thái INS và bù vận tốc cánh tay đòn trong mô hình DVL. Vì vậy, nhiễu ở sai số trạng thái dự đoán và sai số phép đo DVL có thể tương quan.

Hãy phân biệt:

- Test truyền phương sai gyro độc lập ở C3.
- Test innovation và NIS của bộ lọc tích hợp.

Trong test tích hợp, lưu residual, H, R, S và trạng thái chấp nhận của từng phép đo DVL trước khi gating. Kiểm tra residual chuẩn hóa, phân phối NIS và tương quan theo thời gian. Không chỉ đánh giá các phép đo được chấp nhận.

Nếu chỉ cộng phương sai gyro vào R chưa đủ do tương quan, hãy trình bày bằng chứng và đề xuất cách mô hình hóa hiệp phương sai chéo. Không tự động thay đổi cấu trúc Kalman nếu chưa có kết quả chứng minh.

## D. Đầu ra cần có

Hãy tạo các file test MATLAB riêng, không đặt toàn bộ kiểm thử vào vòng lặp Monte Carlo chính:

- `test_esekf19_jacobian_reset.m`
- `test_esekf19_dvl_leverarm.m`
- Nếu cần, các hàm phụ trợ nhỏ được dùng trực tiếp bởi bộ lọc và test.

Cho phép tách các phép tính thành hàm phụ trợ để test, nhưng phải giữ nguyên kết quả và giao diện của `esekf19_step` với code cũ khi không yêu cầu đầu ra chẩn đoán.

Khi thực thi, in báo cáo theo bảng:

| ID | Test | Metric | Threshold | Actual | PASS/FAIL |
|---|---|---|---|---|---|

Mỗi test phải cung cấp đủ số liệu để người khác có thể xác minh, gồm seed, số mẫu, cấu hình, đơn vị, sai lệch lớn nhất, vị trí phát sinh sai lệch và lý do kết luận.

Lưu kết quả vào file `.mat` và xuất thêm báo cáo `.txt` hoặc `.csv`.

Tạo thêm phần tổng kết:

1. Những phép thử PASS.
2. Những phép thử FAIL.
3. Những phép thử WARN và nguyên nhân.
4. Các sai khác toán học đã được chứng minh.
5. Các xấp xỉ được giữ lại cùng độ lớn sai số.
6. Mã nguồn nào đã thay đổi và lý do thay đổi.
7. Những phép thử chưa chạy do thiếu môi trường hoặc công cụ.

Nếu có MATLAB, hãy chạy thật các test và cung cấp log. Nếu không có MATLAB, hãy chỉ bàn giao mã test và phân tích tĩnh; không được tự điền số liệu giả hoặc tuyên bố PASS.

Không chỉnh Q/P, không điều chỉnh ngưỡng để che lỗi, và không sửa công thức sản xuất trước khi test hiện trạng có kết quả. Ưu tiên phát hiện lỗi và đưa bằng chứng trước khi đề xuất sửa.