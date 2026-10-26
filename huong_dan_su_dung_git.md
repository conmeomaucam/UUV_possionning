# CẨM NANG HƯỚNG DẪN SỬ DỤNG GIT & GITHUB QUẢN LÝ DỰ ÁN CODE HIỆU QUẢ

Tài liệu này được biên soạn ngắn gọn, dễ hiểu và thực chiến, giúp bạn làm chủ Git để quản lý mã nguồn thuật toán, mô phỏng MATLAB và đồng bộ an toàn lên GitHub.

---

## I. NGUYÊN LÝ HOẠT ĐỘNG CỦA GIT (3 TẦNG LÀM VIỆC)

Để không bao giờ bị nhầm lẫn giữa `commit` và `push`, bạn chỉ cần nhớ chu trình 3 tầng:

```
[Thư mục làm việc]  ---> (git add) --->  [Vùng chờ Staging]  ---> (git commit) --->  [Kho lưu trữ Cục bộ]  ---> (git push) --->  [GitHub Đám mây]
 (Working Directory)                       (Index/Staging)                            (Local Repo trên máy)                         (Remote Repo)
```

1. **Thư mục làm việc (Working Dir):** Nơi bạn viết code, chỉnh sửa các file `.m`, `.md`.
2. **Vùng chờ (Staging Area):** Nơi bạn chọn lọc những file đã sẵn sàng để đóng gói (`git add`).
3. **Kho lưu trữ trên máy (Local Repo):** Nơi gói code được đóng dấu lưu trữ vĩnh viễn trên máy tính của bạn (`git commit`).
4. **Kho lưu trữ GitHub (Remote Repo):** Nơi đẩy bản sao lên mạng Internet để bảo vệ dữ liệu và chia sẻ (`git push`).

---

## II. QUY TRÌNH 4 BƯỚC THẦN THÁNH HÀNG NGÀY

Bất cứ khi nào bạn viết xong một tính năng hoặc sửa xong một lỗi, hãy thực hiện theo đúng 4 bước sau:

### Bước 1: Kiểm tra trạng thái thay đổi
Trước khi làm bất cứ việc gì, hãy xem mình đã sửa những file nào:
```bash
git status
```
* File màu đỏ: File đã sửa nhưng chưa đưa vào vùng chờ.
* File màu xanh: File đã sẵn sàng để commit.

### Bước 2: Đưa các file đã sửa vào vùng chờ
* Nếu muốn đưa toàn bộ các file thay đổi vào:
  ```bash
  git add .
  ```
* Nếu chỉ muốn chọn một file cụ thể:
  ```bash
  git add Sensor/Bar_10XT.m
  ```

### Bước 3: Đóng gói và ghi chú nội dung thay đổi (Commit)
Ghi một câu ngắn gọn mô tả bạn vừa làm gì:
```bash
git commit -m "hoàn thiện mô hình cảm biến Bar10XT và tách bạch tham số"
```
*(Lúc này code đã được lưu an toàn 100% trong lịch sử máy tính của bạn).*

### Bước 4: Đẩy lên GitHub
Đẩy các commit trên máy lên kho GitHub:
```bash
git push origin main
```
*(hoặc gõ ngắn gọn `git push`)*

---

## III. CÁCH ĐẶT TÊN COMMIT CHUẨN KỸ SƯ (CONVENTIONAL COMMITS)

Để lịch sử code nhìn chuyên nghiệp và dễ tìm lại sau này, hãy thêm tiền tố ngắn ở đầu commit:

* **feat:** Thêm tính năng mới hoặc cảm biến mới.
  * Ví dụ: `git commit -m "feat: thêm mô hình cảm biến đo sâu Bar10XT"`
* **fix:** Sửa lỗi thuật toán hoặc lỗi cú pháp.
  * Ví dụ: `git commit -m "fix: sửa lỗi trôi bias trong hàm bar10xt_step"`
* **refactor:** Tái cấu trúc, dọn dẹp code sạch hơn nhưng không đổi tính năng.
  * Ví dụ: `git commit -m "refactor: tách hàm DVL_A125 ra file riêng biệt"`
* **docs:** Thêm hoặc sửa file tài liệu `.md`, hướng dẫn.
  * Ví dụ: `git commit -m "docs: cập nhật tài liệu phản biện Bar10XT"`
* **test:** Viết thêm file test hoặc kịch bản mô phỏng kiểm thử.
  * Ví dụ: `git commit -m "test: chạy kiểm thử Monte Carlo 200 lượt cho ES-EKF"`

---

## IV. CÁC LỆNH "CỨU NGUY" KHI GẶP SỰ CỐ THƯỜNG GẶP

### 1. Lỡ sửa code linh tinh, muốn khôi phục lại file gốc chưa sửa
Nếu bạn nghịch thử file mà bị lỗi, muốn quay lại trạng thái lưu gần nhất:
```bash
git restore Sensor/Bar_10XT.m
```

### 2. Lỡ gõ `git add .` nhầm một file không muốn
Nếu bạn muốn bỏ một file ra khỏi danh sách chuẩn bị commit:
```bash
git restore --staged <tên_file>
```

### 3. Xem chi tiết mình vừa sửa những dòng nào so với bản cũ
```bash
git diff
```

### 4. Xem lại lịch sử các commit gần đây
```bash
git log --oneline -n 5
```
*(Hiển thị 5 commit gần nhất trên một dòng ngắn gọn).*

### 5. Cập nhật code mới nhất từ GitHub về máy
Khi bạn sửa code trên web GitHub hoặc làm việc trên máy khác, muốn kéo code mới nhất về:
```bash
git pull origin main
```

---

## V. ĐẶC THÙ KHI QUẢN LÝ DỰ ÁN MATLAB VỚI GIT

### 1. Tránh đẩy file rác của MATLAB lên GitHub (`.gitignore`)
MATLAB tự động tạo ra rất nhiều file tạm và cache. Đảm bảo file `.gitignore` trong thư mục dự án có chứa các dòng sau:
```gitignore
# File sao lưu tự động của MATLAB
*.asv
*.m~

# Thư mục cache Simulink và bộ dịch
slprj/
*.slxc

# File kết quả mô phỏng quá nặng (nếu không cần lưu lên git)
*.mat
```

### 2. Xử lý Submodule thư mục `AUV_simulation/MSS`
Trong dự án của bạn, thư mục `AUV_simulation/MSS` là một Git Submodule (kho code con của GS. Fossen). Khi bạn chạy lệnh `git status`, thỉnh thoảng sẽ thấy dòng:
```
modified:   AUV_simulation/MSS (modified content, untracked content)
```
* **Lý do:** Bạn có chỉnh sửa file hoặc sinh ra file rác bên trong thư mục `MSS`.
* **Cách xử lý:** 
  - Nếu chỉ sửa các file ở ngoài (như `Sensor/`, `TEST/`, `algo/`), bạn không cần quan tâm đến cảnh báo này.
  - Khi commit chỉ cần chỉ định các file mình làm việc:
    ```bash
    git add Sensor/ algo/ TEST/ *.m *.md
    git commit -m "nội dung commit"
    git push
    ```

---

## VI. BẢNG TRA CỨU NHANH (CHEAT SHEET)

| Thao tác | Câu lệnh Git | Ghi chú |
| :--- | :--- | :--- |
| **Kiểm tra trạng thái** | `git status` | Nên gõ đầu tiên trước mỗi thao tác |
| **Đưa tất cả vào hàng chờ** | `git add .` | Chuẩn bị đóng gói toàn bộ thay đổi |
| **Đưa 1 file vào hàng chờ** | `git add <đường_dẫn_file>` | Chỉ đóng gói file chỉ định |
| **Lưu đóng gói trên máy** | `git commit -m "mô tả"` | Lưu trữ vào lịch sử cục bộ |
| **Đẩy lên GitHub** | `git push origin main` | Đồng bộ bản lưu lên đám mây |
| **Kéo từ GitHub về máy** | `git pull origin main` | Cập nhật thay đổi mới nhất từ mạng |
| **Xem lịch sử ngắn gọn** | `git log --oneline -n 5` | Hiển thị 5 commit gần nhất |
| **Hủy sửa đổi ở file** | `git restore <file>` | Khôi phục file về trạng thái commit trước |
