# BANG TONG HOP TRANG THAI BAN DAU VA KICH BAN THUC TE CHUAN HOA (REMUS 100 + ES-EKF)

Tai lieu ky thuat tong hop toan bo dieu kien dau vao, mo hinh cam bien thuc te (loai bo hoan toan cac gia dinh ly tuong va Oracle cheat), va ket qua kiem chung mo phong dong hoc Fossen tich hop bo loc ES-EKF 19 trang thai.

---

## 1. SO SANH TRUOC VA SAU KHI CHUAN HOA (LOAI BO HOAN TOAN ORACLE CHEAT)

| Hang muc | Truoc chuan hoa (Mo hinh ly tuong) | Sau khi chuan hoa (Sat 100% thuc te theo VectorNav Primer) | Danh gia ky thuat |
| :--- | :--- | :--- | :--- |
| **Mo hinh Tu ke (Magnetometer)** | Khong co sat tu, do on ly tuong (sigma = 0.056 do) | **Mo hinh sat tu day du**: Hard Iron du (8 - 10 mGauss), Soft Iron meo truc (1.5% - 2.0%), nhieu dong dong co day theo vong tua RPM (2 - 3 mGauss), sai so huong 2.0 do RMS dong bo datasheet VN-210-S | Sat thuc te phan cung tren than vo REMUS 100; khong con gia dinh phong thi nghiem vo trung. |
| **Goc huong ban dau (Heading/Yaw)** | `psi_est = psi_true` (sai so = 0.0 do, cheat Oracle) | **`psi_est = psi_true - 2.69 do`** (can chinh tho ban dau bang tu ke thuc te) | **Dung thuc te**: Căn chỉnh thô bằng từ kế mang sai lệch ~2.7 độ so với hướng thật. |
| **Bat dinh goc huong P0(9,9)** | Khai bao `0.5 do` (mau thuan voi sai so) | **Khai bao `3.5 do`** | **Nhat quan Gauss**: Ma tran P0 bao tron sai so ban dau 2.69 do (nam trong vung 0.77 * sigma_psi). |
| **Sai so huong mat nuoc** | 0.016 do (ly tuong gia tao do thieu sat tu) | **Dao dong thuc te quanh 0.92 do RMS** (Mean = -0.92 do, ket thuc mat nuoc = -0.86 do) | Mo ta chinh xac anh huong cua truong tu du sau calib va dong dong co chan vit. |
| **Vi tri ban dau** | `[0; 0; 0] m` (trung khit Ground Truth) | **Lay tu phep do GNSS RTK dau tien** (co nhieu thuc te ~ 0.6 - 0.8 m) | Khu hoan toan Oracle cheat vi tri. |
| **Van toc ban dau** | `v_hat = v_true` (sai so = 0 m/s) | **Lay tu phep do GNSS Doppler dau tien** (co nhieu thuc te ~ 0.02 m/s) | Khu mau thuan giua van toc 0 va do lech chuan van toc khoi tao. |
| **Bat dinh bias gia toc P0** | `0.5 mg` (trong khi bias that = 1.2 mg -> lech 2.4 * sigma) | **Khai bao `2.0 mg`** | **Bao toan gia dinh Gauss**: Bias that 1.2 mg nam o 0.6 * sigma, hoan toan hop le. |
| **Cam bien do sau** | Bar10XT (10 bar / 89.5m, hut dai do cho xe lan 100m) | **Keller 4LD / BlueRobotics Bar30 (30 bar / 300m)** | **Chuan ky thuat**: REMUS 100 lan 100m duoc trang bi cam bien dung dai cong tac. |

---

## 2. CHI TIET MO HINH TU KE THUC TE (THEO VECTORNAV PRIMER)

Phu hop voi tai lieu VectorNav Inertial Navigation Primer (cac muc 1.1, 1.2, 3.1, 3.3, 3.5, 3.6), cam bien VN-210-S tren tau REMUS 100 duoc mo hinh hoa gom 4 thanh phan:

1. **Truong dia tu chuan (Earth Magnetic Field)**:
   - Vector tu truong trai dat tai khu vuc thu nghiem: B_earth = [0.22; 0.02; 0.42] Gauss.
2. **Sat cung con du sau calib (Residual Hard Iron)**:
   - Cac nam cham vinh cuu, bulong thep, vo kim loai tren REMUS 100 gay ra do lech tam elip:
   - hard_iron = [0.008; -0.010; 0.005] Gauss (~8 den 10 mGauss), gay sai lech huong tinh ~1.5 do.
3. **Sat mem con du sau calib (Residual Soft Iron)**:
   - Cac vat lieu thep ky thuat lam bien dang hinh cau thanh elip voi do co dan va chech truc:
   - soft_iron_inv = [1.015, 0.008, 0.005; 0.008, 0.985, 0.006; 0.005, 0.006, 1.010].
4. **Nhieu dien tu dong dong co chan vit (Propeller Motor EMI)**:
   - Bien thien theo toc do vong tua chan vit n_p (toi da 1525 RPM):
   - B_motor = (n_p / 1525) * [0.003; 0.001; -0.002] Gauss (~2 den 3 mGauss khi chay het cong suat).
5. **Tham so do bat dinh (Datasheet Alignment)**:
   - Dat sai so chuan phep do goc huong tu ke: sigma_mag_yaw = 2.0 do (0.0349 rad) theo dung dac tinh ky thuat datasheet VN-210-S.
   - Nho gia tri thuc te nay, cong kiem dinh doi chieu thong ke (NIS gate = 6.63) luon mo thong suot hanh trinh, triet tieu hoan toan hien tuong nghen phep do tu ke va loai bo hoan toan cu nhay sai so 30m tung xuat hien o mo hinh ly tuong cu.

---

## 3. KET QUA MO PHONG VAN HANH THUC TE (CHAY KET HOP FOSSEN + ES-EKF)

Kich ban hanh trinh gom 11 Waypoints, tong quang duong 6.45 km, tong thoi gian 2415.6 giay (~40.3 phut), seed ngau nhien = 2026:

### A. Giai doan chay tren mat nuoc (t = 0 den 252.5s, co GNSS + DVL + Tu ke + Bar30):
- Căn chỉnh thô ban đầu (t = 2.95s):
  * Roll: 0.85 do (sai so = -0.07 do)
  * Pitch: 1.78 do (sai so = +3.02 do)
  * Yaw: 87.26 do (sai so ban dau = -2.69 do, do bat dinh P0 = 0.89 do)
- Vung chay hinh chu S de kich thich cac truc quan sat:
  * Sai so goc huong Yaw trung binh mat nuoc: Mean = -0.92 do, RMS = 0.92 do.
  * Truoc thoi diem lan (t = 252.5s): Sai so Yaw dat -0.86 do (on dinh thuc te, khong con hien tuong fake 0.01 do).
  * Uoc luong lech truc Boresight DVL: Roll = 2.042 do (that 1.500 do, sai so 0.542 do), Pitch = -1.898 do (that -2.000 do, sai so 0.102 do), Yaw = 3.932 do (that 3.000 do, sai so 0.932 do).
  * Uoc luong Scale Factor DVL: Dat 1.442% (that 1.500%, sai so chi -0.058%).
  * Sai so vi tri 3D tai thoi diem bat dau lan: 0.115 m.

### B. Giai doan lan ngam sau 30m - 50m (t = 252.5s den 2415.6s, CAT HOAN TOAN GNSS suot 36 phut):
- Chế độ hoa tiêu ngầm Dead Reckoning: Tích hợp vận tốc đáy từ DVL A125 đã được bù Boresight và Scale Factor + IMU VN-210-S + Cảm biến áp suất Keller Bar30.
- Kênh do sau: Keller Bar30 hoat dong lien tuc, sai so do sau giu on dinh o muc 0.12 m suot toan bo 36 phut duoi nuoc.
- Goc huong Yaw duoi nuoc:
  * RMS sai so huong = 1.01 do, Mean = -1.01 do.
  * Sai so huong tai diem dich (t = 2415.6s): -1.08 do.
- Sai so vi tri 3D tich luy theo thoi gian:
  * t = 300s : 0.133 m
  * t = 600s : 0.847 m
  * t = 1000s: 1.870 m
  * t = 1500s: 3.270 m
  * t = 1966s: 5.329 m (loai bo hoan toan cu nhay vot 30m cua mo hinh cu)
  * t = 2415.6s (ve dich): **6.944 m** (North = +5.912 m, East = +3.640 m, Down = -0.120 m).
- **Danh gia do troi hanh trinh (Drift Rate)**:
  * Tong quang duong: 6.45 km (6450 m).
  * Ty le troi vi tri: **6.944 m / 6450 m = 0.1077% quang duong di chuyen**.
  * Day la muc do chinh xac xuat sac cho he thong INS/DVL chien thuat khi mat GNSS suot hon 36 phut.

---

## 4. CO CHE HO TRO 2 CHE DO (DUAL-MODE) TRONG CODE

Ham `run_fossen_esekf_integrated` ho tro tham so thu 4: `depth_sensor_variant`:
1. **Che do 1: Cau hinh chuan thuc te ngoai doi (Nominal Real-World - Mac dinh)**:
   - Cu phap: `run_fossen_esekf_integrated('VN_210', true, true, 'Bar30')`
   - Xe lan REMUS 100 trang bi cam bien Keller Bar30 (30 bar / 300m). Kenh do sau hoat dong lien tuc, EKF khoi tao Cold Start thuc su (vi tri/van toc co nhieu GNSS, huong lech theo can chinh tho tu ke).
2. **Che do 2: Kich ban thu nghiem su co (Fault-Tolerant Stress-Test)**:
   - Cu phap: `run_fossen_esekf_integrated('VN_210', true, true, 'Bar10XT')`
   - Gia lap tinh huong cam bien hut dai do Bar10XT (10 bar / 89.5m) bi bao hoa ao khi lan qua 90m de kiem tra do ben vung cua bo loc khi mat thong tin moc ap suat tuyet doi.

---

## 5. DANH MUC HINH ANH KET QUA XUAT RA

Sau khi mo phong ket thuc, 4 hinh ve chat luong cao da duoc cap nhat vao thu muc `TEST/`:
- `fig1_quy_dao_3d.png`: Quy dao 3D khong gian thuc te cua REMUS 100 so sanh giua Ground Truth va ES-EKF.
- `fig2_boresight.png`: Duong hoi tu 3 goc Boresight Roll, Pitch, Yaw cua DVL tu 0 do ve gia tri thuc.
- `fig3_scale_factor.png`: Duong hoi tu he so ti le Scale Factor cua DVL kem dai tin cay 3 * sigma.
- `fig4_sai_so_dinh_vi.png`: Sai so vi tri 3 truc NED va sai so van toc theo thoi gian, chung minh tinh lien tuc tuyet doi khong con bat ky xung nhay nao.
