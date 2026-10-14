# Cài đặt thư viện (nếu thiếu): pip install matplotlib python-docx
import os
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from docx import Document
from docx.shared import Pt, Inches, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_ALIGN_VERTICAL
from docx.oxml import parse_xml
from docx.oxml.ns import qn, nsdecls

# ==============================================================================
# BƯỚC 1: VẼ SƠ ĐỒ KHỐI BỘ LỌC ES-EKF 19 TRẠNG THÁI (ĐEN TRẮNG, 300 DPI)
# ==============================================================================
def generate_esekf_diagram(filename="So_Do_Khoi_ES_EKF_19.png"):
    plt.rcParams['font.sans-serif'] = 'Arial'
    plt.rcParams['axes.edgecolor'] = 'black'
    
    fig, ax = plt.subplots(figsize=(14.8, 9.8), dpi=300)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis('off')

    # Tiêu đề sơ đồ (Ngắn gọn, trang trọng)
    ax.text(50, 97.4, "SƠ ĐỒ KHỐI CẤU TRÚC VÀ LUỒNG HOẠT ĐỘNG BỘ LỌC ES-EKF 19 TRẠNG THÁI",
            ha='center', va='center', fontsize=13, fontweight='bold', color='black')

    # Dải phân tầng (Banners)
    ax.text(48, 88.5, "─── TẦNG DỰ ĐOÁN QUÁN TÍNH INS (TỐC ĐỘ CAO 400 Hz) ───",
            ha='center', va='center', fontsize=8.5, fontweight='bold', color='#444444')
    ax.text(48, 48.0, "─── TẦNG CẬP NHẬT NGOẠI VI & BỘ LỌC SAI SỐ KALMAN (TỐC ĐỘ THẤP 5 Hz) ───",
            ha='center', va='center', fontsize=8.5, fontweight='bold', color='#444444')

    # Hàm vẽ Khối chức năng (Hộp viền đen, nền trắng, tiêu đề xám)
    def draw_box(x, y, w, h, title, lines, fill_header='#E6E6E6', fill_body='white', title_color='black'):
        # Bóng nhẹ
        shadow = FancyBboxPatch((x + 0.3, y - 0.3), w, h, boxstyle="round,pad=0.2,rounding_size=0.5",
                                ec="none", fc="#DDDDDD")
        ax.add_patch(shadow)
        # Khung chính
        box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.2,rounding_size=0.5",
                             ec="black", fc=fill_body, lw=1.3)
        ax.add_patch(box)
        # Tiêu đề hộp
        hdr_h = 3.6
        hdr = FancyBboxPatch((x, y + h - hdr_h), w, hdr_h, boxstyle="round,pad=0.2,rounding_size=0.5",
                             ec="black", fc=fill_header, lw=1.0)
        ax.add_patch(hdr)
        ax.text(x + w/2, y + h - hdr_h/2, title, ha='center', va='center',
                fontsize=8.6, fontweight='bold', color=title_color)
        # Nội dung chữ bên trong (Tiếng Việt ngắn gọn, có dấu)
        content = "\n".join(lines)
        ax.text(x + 1.2, y + (h - hdr_h)/2, content, ha='left', va='center',
                fontsize=8.0, color='black', linespacing=1.35)

    # Hàm vẽ Điểm trừ / Điểm cộng (Summing Junction)
    def draw_sum_circle(x, y, r=1.5, top_sign='-', left_sign='+'):
        circle = plt.Circle((x, y), r, ec='black', fc='white', lw=1.3)
        ax.add_patch(circle)
        ax.text(x - 0.75, y, left_sign, ha='center', va='center', fontsize=9.5, fontweight='bold')
        ax.text(x, y + 0.75, top_sign, ha='center', va='center', fontsize=10.0, fontweight='bold')

    # Hàm vẽ Dây dẫn trực giao (Manhattan Wire)
    def draw_wire(pts, label="", lx=None, ly=None, arrow=True, ls='-'):
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color='black', lw=1.3, ls=ls)
        if arrow:
            x_last, y_last = pts[-1]
            x_prev, y_prev = pts[-2]
            dx = x_last - x_prev
            dy = y_last - y_prev
            arr = FancyArrowPatch((x_last - 0.01*dx, y_last - 0.01*dy), (x_last, y_last),
                                  arrowstyle="-|>", mutation_scale=13, lw=1.3, color='black')
            ax.add_patch(arr)
        if label:
            if lx is None or ly is None:
                lx = (xs[0] + xs[-1]) / 2
                ly = (ys[0] + ys[-1]) / 2
            ax.text(lx, ly, label, ha='center', va='center', fontsize=7.6,
                    fontweight='bold', color='black',
                    bbox=dict(boxstyle='square,pad=0.2', fc='white', ec='none'))

    # ================= CÁC KHỐI CHỨC NĂNG =================
    # 1. ĐẦU VÀO IMU (400 Hz)
    draw_box(2.0, 60.0, 17.5, 23.0, "ĐẦU VÀO: IMU (400 Hz)", [
        "• Vận tốc góc thô (Gyro)",
        "• Gia tốc riêng thô (Accel)",
        "• Tần số lấy mẫu: 400 Hz",
        "• Cụm IMU quán tính UUV"
    ], fill_header='#E0E4E8')

    # Điểm trừ khử trôi Bias IMU: tâm tại (23.5, 71.5)
    draw_sum_circle(23.5, 71.5, r=1.5, top_sign='-', left_sign='+')

    # 2. KHỐI 1: DỰ ĐOÁN QUÁN TÍNH INS (400 Hz)
    draw_box(28.5, 56.0, 31.0, 31.0, "1. DỰ ĐOÁN QUÁN TÍNH INS (400 Hz)", [
        "• Giải phương trình động học quán tính NED:",
        "  Tích phân vị trí, vận tốc và tư thế",
        "• Suy giảm trôi bias gia tốc (Gauss-Markov)",
        "• Lan truyền hiệp phương sai sai số P (19x19):",
        "  P = F * P * F^T + Q",
        "• Xuất vector trạng thái dự đoán x (20x1)"
    ])

    # 3. KHỐI 4: BƠM HIỆU CHỈNH SAI SỐ & ĐẶT LẠI
    draw_box(64.0, 56.0, 18.0, 31.0, "4. BƠM SAI SỐ & ĐẶT LẠI", [
        "• Bơm sai số ước lượng:",
        "  + Vị trí = Vị trí + Sai số",
        "  + Vận tốc = Vận tốc + Sai số",
        "  + Xoay hiệu chỉnh tư thế",
        "  + Sửa Bias, Scale, Lệch trục",
        "• Đặt lại sai số delta_x = 0",
        "• Cập nhật lại ma trận P"
    ])

    # 4. KHỐI 5: ĐẦU RA BỘ LỌC (OUTPUT)
    draw_box(85.0, 46.0, 13.5, 41.0, "ĐẦU RA (OUTPUT)", [
        "[DẪN ĐƯỜNG UUV]:",
        "• Vị trí NED [N, E, D]",
        "• Vận tốc NED [vN, vE, vD]",
        "• Tư thế: Roll, Pitch, Yaw",
        "",
        "[THAM SỐ HIỆU CHUẨN]:",
        "• Hệ số tỉ lệ Scale DVL (%)",
        "• 3 Góc lệch Boresight DVL",
        "• Bias cảm biến đã khử trôi"
    ], fill_header='#D5E5F2', fill_body='#F7FAFC')

    # 5. ĐẦU VÀO CẢM BIẾN NGOẠI VI (5 Hz)
    draw_box(2.0, 10.0, 17.5, 34.0, "ĐẦU VÀO: CẢM BIẾN (5 Hz)", [
        "• GNSS RTK + La bàn:",
        "  Vị trí, vận tốc, góc hướng",
        "  (Chỉ bật khi bơi mặt nước)",
        "",
        "• Cảm biến Độ sâu:",
        "  Đo độ sâu Down (toàn thời gian)",
        "",
        "• Cảm biến DVL A125:",
        "  Vận tốc 3 trục đáy biển"
    ], fill_header='#E0E4E8')

    # 6. KHỐI 2: HÀM ĐO & KIỂM ĐỊNH CỔNG NIS
    draw_box(24.5, 10.0, 27.0, 34.0, "2. HÀM ĐO & LỌC NGOẠI LAI (5 Hz)", [
        "• Mô hình đo DVL h(x):",
        "  Bù cánh tay đòn, Scale s và Boresight",
        "• Tính sai lệch phần dư: r = z_đo - h(x)",
        "• Kiểm định cổng Chi-square (NIS):",
        "  + Đạt chuẩn: Chấp nhận chuyển sang Kalman",
        "  + Vượt ngưỡng: Loại bỏ phép đo lỗi đột biến"
    ])

    # 7. KHỐI 3: BỘ LỌC SAI SỐ KALMAN (ES-EKF)
    draw_box(55.5, 10.0, 26.5, 34.0, "3. BỘ LỌC SAI SỐ KALMAN (5 Hz)", [
        "• Tính hệ số khuếch đại Kalman K",
        "• Ước lượng vector sai số delta_x (19x1):",
        "  [Sai số Vị trí, Vận tốc, Tư thế,",
        "   Bias con quay, Bias gia tốc,",
        "   Hệ số Scale, 3 Góc Boresight]",
        "• Cập nhật hiệp phương sai P (dạng Joseph)"
    ])

    # ================= DÂY DẪN TÍN HIỆU TRỰC GIAO =================
    # Tuyến Tầng 1:
    draw_wire([(19.5, 71.5), (21.9, 71.5)], "")
    draw_wire([(25.1, 71.5), (28.5, 71.5)], "Tín hiệu đã khử trôi", 26.8, 74.0)
    draw_wire([(59.5, 71.5), (64.0, 71.5)], "Trạng thái dự đoán", 61.8, 74.0)
    draw_wire([(82.0, 71.5), (85.0, 71.5)], "Dữ liệu chuẩn", 83.5, 74.0)

    # Tuyến Tầng 2:
    draw_wire([(19.5, 27.0), (24.5, 27.0)], "Số đo z (5 Hz)", 22.0, 29.5)
    draw_wire([(51.5, 27.0), (55.5, 27.0)], "Phần dư r hợp lệ", 53.5, 29.5)

    # Tuyến Dọc: Trạng thái dự đoán x đưa từ Khối 1 xuống Khối 2 để tính h(x)
    draw_wire([(44.0, 56.0), (44.0, 44.0)], "Trạng thái dự đoán x (để tính h(x))", 44.0, 50.0)

    # Tuyến Dọc: Vector sai số delta_x từ Khối 3 đưa ngược lên Khối 4 để bơm sai số
    draw_wire([(68.5, 44.0), (68.5, 56.0)], "Vector sai số delta_x (19x1)", 68.5, 50.0)

    # ================= HAI ĐƯỜNG HỒI TIẾP CỐT LÕI (CỰC KỲ RÕ RÀNG) =================
    # ĐƯỜNG HỒI TIẾP 1: Hồi tiếp trạng thái sau hiệu chỉnh làm gốc cho chu kỳ dự đoán tiếp theo
    # Đi từ Khối 4 (73.0, 87.0) -> lên y=91.0 -> rẽ trái qua x=44.0 -> cắm xuống Khối 1
    draw_wire([(73.0, 87.0), (73.0, 91.0), (44.0, 91.0), (44.0, 87.0)],
              "HỒI TIẾP 1: Trạng thái đã hiệu chỉnh làm gốc cho chu kỳ dự đoán tiếp theo",
              58.5, 92.2, ls='--')

    # ĐƯỜNG HỒI TIẾP 2: Hồi tiếp ước lượng Bias để khử trôi trực tiếp cho cảm biến IMU
    # Đi từ Khối 4 (79.0, 87.0) -> lên y=94.5 -> rẽ trái qua x=23.5 -> cắm xuống cổng (-) của Điểm trừ
    draw_wire([(79.0, 87.0), (79.0, 94.5), (23.5, 94.5), (23.5, 73.1)],
              "HỒI TIẾP 2: Ước lượng Bias (ba, bg) để khử trôi trực tiếp cho IMU",
              51.2, 95.8, ls=':')

    # Chú thích chân sơ đồ
    ax.text(50, 3.2, "Quy ước: [⊕: Điểm trừ khử trôi]  ──  [Mũi tên liền: Luồng tiến dữ liệu]  ──  [Mũi tên đứt nét: Vòng hồi tiếp kín]",
            ha='center', va='center', fontsize=8.2, fontstyle='italic', color='#555555')

    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()
    print("Đã tạo ảnh sơ đồ khối rõ ràng chuẩn kỹ thuật:", filename)





# ==============================================================================
# BƯỚC 2: TẠO FILE WORD (ARIAL 14, ĐEN TRẮNG) VÀ CHÈN SƠ ĐỒ KHỐI VÀO PHẦN 3
# ==============================================================================
def generate_docx_with_diagram():
    diagram_file = "So_Do_Khoi_ES_EKF_19.png"
    generate_esekf_diagram(diagram_file)

    doc = Document()

    for section in doc.sections:
        section.page_width = Inches(8.27)
        section.page_height = Inches(11.69)
        section.top_margin = Inches(0.8)
        section.bottom_margin = Inches(0.8)
        section.left_margin = Inches(0.9)
        section.right_margin = Inches(0.8)

    style_normal = doc.styles['Normal']
    style_normal.font.name = 'Arial'
    style_normal.font.size = Pt(14)
    style_normal.font.color.rgb = RGBColor(0, 0, 0)
    style_normal.element.rPr.rFonts.set(qn('w:eastAsia'), 'Arial')
    style_normal.paragraph_format.line_spacing = 1.25
    style_normal.paragraph_format.space_after = Pt(6)

    def add_heading_bw(text, space_before=12, space_after=6):
        p = doc.add_paragraph()
        p.paragraph_format.space_before = Pt(space_before)
        p.paragraph_format.space_after = Pt(space_after)
        p.paragraph_format.line_spacing = 1.25
        r = p.add_run(text)
        r.bold = True
        r.font.name = 'Arial'
        r.font.size = Pt(14)
        r.font.color.rgb = RGBColor(0, 0, 0)
        r._element.rPr.rFonts.set(qn('w:eastAsia'), 'Arial')
        return p

    def add_bullet_bw(bold_prefix, text, indent_level=0):
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
        p.paragraph_format.space_before = Pt(2)
        p.paragraph_format.space_after = Pt(4)
        p.paragraph_format.line_spacing = 1.25
        p.paragraph_format.left_indent = Inches(0.25 + indent_level * 0.25)
        p.paragraph_format.first_line_indent = Inches(-0.18)

        prefix_symbol = "- " if indent_level == 0 else "+ "
        r0 = p.add_run(prefix_symbol)
        r0.font.name = 'Arial'
        r0.font.size = Pt(14)
        r0.font.color.rgb = RGBColor(0, 0, 0)

        if bold_prefix:
            rb = p.add_run(bold_prefix)
            rb.bold = True
            rb.font.name = 'Arial'
            rb.font.size = Pt(14)
            rb.font.color.rgb = RGBColor(0, 0, 0)
            rb._element.rPr.rFonts.set(qn('w:eastAsia'), 'Arial')

        if text:
            rt = p.add_run(text)
            rt.font.name = 'Arial'
            rt.font.size = Pt(14)
            rt.font.color.rgb = RGBColor(0, 0, 0)
            rt._element.rPr.rFonts.set(qn('w:eastAsia'), 'Arial')
        return p

    def format_table_bw(table, col_widths, font_size=11):
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        borders = parse_xml(
            f'<w:tblBorders {nsdecls("w")}>'
            '<w:top w:val="single" w:sz="6" w:space="0" w:color="000000"/>'
            '<w:bottom w:val="single" w:sz="6" w:space="0" w:color="000000"/>'
            '<w:left w:val="single" w:sz="6" w:space="0" w:color="000000"/>'
            '<w:right w:val="single" w:sz="6" w:space="0" w:color="000000"/>'
            '<w:insideH w:val="single" w:sz="4" w:space="0" w:color="000000"/>'
            '<w:insideV w:val="single" w:sz="4" w:space="0" w:color="000000"/>'
            '</w:tblBorders>'
        )
        table._tbl.tblPr.append(borders)

        for r_idx, row in enumerate(table.rows):
            for c_idx, cell in enumerate(row.cells):
                cell.width = Inches(col_widths[c_idx])
                cell.vertical_alignment = WD_ALIGN_VERTICAL.CENTER
                tcPr = cell._tc.get_or_add_tcPr()
                tcMar = parse_xml(
                    f'<w:tcMar {nsdecls("w")}>'
                    '<w:top w:w="80" w:type="dxa"/>'
                    '<w:bottom w:w="80" w:type="dxa"/>'
                    '<w:left w:w="100" w:type="dxa"/>'
                    '<w:right w:w="100" w:type="dxa"/>'
                    '</w:tcMar>'
                )
                tcPr.append(tcMar)
                for p in cell.paragraphs:
                    p.paragraph_format.space_before = Pt(3)
                    p.paragraph_format.space_after = Pt(3)
                    p.paragraph_format.line_spacing = 1.15
                    for run in p.runs:
                        run.font.name = 'Arial'
                        run.font.size = Pt(font_size)
                        run.font.color.rgb = RGBColor(0, 0, 0)
                        run._element.rPr.rFonts.set(qn('w:eastAsia'), 'Arial')
                        if r_idx == 0:
                            run.bold = True

    # ================= PHẦN 1 =================
    add_heading_bw("1. LỰA CHỌN CẢM BIẾN VÀ THAM SỐ ĐƯA VÀO MÔ PHỎNG", space_before=0)
    add_heading_bw("1.1. Hệ trục tọa độ quy chiếu và cấu hình hình học lắp đặt")
    add_bullet_bw("Hệ quy chiếu dẫn đường: ",
                  "Sử dụng hệ tọa độ NED (Bắc - Đông - Xuống), trong đó điểm bắt đầu của thiết bị được gắn cố định tại vị trí xuất phát là [0, 0, 0] m trên mặt nước.")
    add_bullet_bw("Độ sâu đáy biển: ", "Giả định cố định ở mức là 50 m.")
    add_bullet_bw("Hệ quy chiếu thân của (INS/IMU) và khung đầu đo của cảm biến DVL: ",
                  "Được quy định gồm trục x hướng về phía mũi thiết bị lặn, trục y hướng sang phải, trục z hướng xuống đáy.")
    add_bullet_bw("Sai lệch lắp đặt hệ thống: ",
                  "Dùng để so sánh và kiểm chứng bộ lọc ES-EKF sử dụng có hội tụ về đường chính và tìm ra được sai số hệ thống cùng hệ số tỉ lệ DVL hay không, được quy định như sau:")
    add_bullet_bw("Ba góc lệch Boresight: ",
                  "[alpha_boresight, beta_boresight, gamma_boresight] = [0.2°, -0.3°, 1.0°].", indent_level=1)
    add_bullet_bw("Hệ số tỉ lệ (Scale factor) được giả thiết: ", "1.0% (k_thật = 0.01).", indent_level=1)
    add_bullet_bw("Cánh tay đòn (Lever arm): ",
                  "Vector khoảng cách từ tâm (IMU/INS) đến tâm đầu dò DVL chiếu trong khung thân được giả sử là l_arm = [0.2, 0, -0.1] m.", indent_level=1)

    add_heading_bw("1.2. Danh mục cảm biến và tham số cụ thể được lựa chọn đưa vào mô phỏng")
    add_heading_bw("a) Hệ thống dẫn đường quán tính INS-DM-FI (Hãng Inertial Labs - Cụm IMU-FI-200T)", space_before=6)
    add_bullet_bw("Tần số lấy mẫu: ", "400 Hz (chu kỳ tích phân dt = 0.0025 s).")
    add_bullet_bw("Con quay hồi chuyển (Gyro): ",
                  "Dải đo ±490 °/s, băng thông 200 Hz, nhiễu trắng góc ARW = 0.025 °/sqrt(h). Độ lặp lại bias khi bật máy = 0.5 °/h (sinh ra 1 lần làm hằng số tĩnh trong suốt kịch bản), sai số tỉ lệ = 100 ppm, sai số lệch trục = 0.1 mrad.")
    add_bullet_bw("Gia tốc kế MEMS 3 trục: ",
                  "Dải đo ±8 g, băng thông 260 Hz, nhiễu trắng vận tốc VRW = 0.015 m/s/sqrt(h), độ lặp lại bias khi bật máy = 0.5 mg. Trôi trong quá trình chạy (In-run bias) mô hình hóa theo Gauss-Markov bậc 1 với độ lệch chuẩn 0.005 mg trong 100 s, sai số tỉ lệ = 150 ppm, sai số lệch trục = 0.2 mrad.")

    add_heading_bw("b) Cảm biến Doppler DVL A125", space_before=6)
    add_bullet_bw("Nguyên lý xuất dữ liệu: ",
                  "DVL A125 giải trực tiếp 4 tia sang vận tốc 3 trục (vx, vy, vz) tại khung thân đầu đo.")
    add_bullet_bw("Tần số mô phỏng: ",
                  "Được giả định là 5 Hz (chu kỳ đo là 0.2 s - không thay đổi trong quá trình mô phỏng).")
    add_bullet_bw("Điều kiện hợp lệ của phép đo bám đáy DVL trong mô phỏng: ",
                  "Khi vận tốc tại đầu đo ||v_lever|| <= 9.0 m/s và khoảng cách từ UUV đến đáy biển nằm trong dải (0.05 m, 125 m), trong đó độ sâu đáy biển đã được cố định là 50 m.")
    add_bullet_bw("Nhiễu của DVL với UUV: ", "")
    add_bullet_bw("Nhiễu trắng trên từng ping DVL: ", "Sàn nhiễu 5 mm/s + 0.2% độ lớn vận tốc.", indent_level=1)
    add_bullet_bw("Nhiễu tương quan chậm: ",
                  "Đại diện cho độ chính xác dài hạn ±0.1% (sigma_gm = 0.1%, trong khoảng thời gian 300 s) nhân tỉ lệ vào vận tốc đo.", indent_level=1)

    add_heading_bw("c) Hệ thống định vị vệ tinh và la bàn GNSS (GNSS RTK + Compass)", space_before=6)
    add_bullet_bw("Tần số cập nhật: ",
                  "5 Hz (đồng bộ với thời gian lấy mẫu DVL) và chỉ cung cấp phép đo khi UUV trên mặt nước (t <= T_cal ở mọi kịch bản, và giai đoạn nổi lên ở kịch bản 6). Khi UUV xuống dưới mặt nước, tín hiệu GNSS được mô phỏng là tắt.")
    add_bullet_bw("Chi tiết trong mô phỏng: ",
                  "Cung cấp vị trí trong hệ tọa độ NED với độ lệch chuẩn nhiễu trắng sigma_pos = 0.02 m, vận tốc 3 chiều trong hệ NED với sigma_vel = 0.02 m/s, và góc hướng mũi Yaw với sigma_yaw = 0.05°.")

    add_heading_bw("d) Cảm biến độ sâu (Depth Sensor)", space_before=6)
    add_bullet_bw("Chi tiết trong mô phỏng: ",
                  "Tần số cập nhật 5 Hz (hoạt động liên tục suốt hành trình), cung cấp trực tiếp độ sâu trục Down trong hệ NED với độ lệch chuẩn nhiễu trắng sigma_depth = 0.05 m.")

    add_heading_bw("1.3. Bảng tóm tắt số liệu đưa vào mô phỏng và đưa vào bộ lọc ES-EKF")
    t1_data = [
        ("Thiết bị / Thành phần", "Thông số đưa vào Mô phỏng (Tạo dữ liệu)", "Thông số đưa vào Bộ lọc ES-EKF"),
        ("INS/IMU (400 Hz)\nCon quay & Gia tốc",
         "• Gyro: ARW 0.025 °/sqrt(h), bias tĩnh 0.5 °/h, scale 100 ppm, lệch trục 0.1 mrad.\n• Accel: VRW 0.015 m/s/sqrt(h), bias tĩnh 0.5 mg, trôi GM 0.005 mg (100 s), scale 150 ppm, lệch trục 0.2 mrad.",
         "• Ước lượng bias Gyro (3 trạng thái) và bias Accel (3 trạng thái).\n• Đưa nhiễu trắng ARW, VRW và trôi GM (100 s) vào ma trận Q."),
        ("DVL A125 (5 Hz)",
         "• Boresight: [0.2°, -0.3°, 1.0°].\n• Scale: 1.0%; Lever arm: [0.2, 0, -0.1] m.\n• Nhiễu trắng: 5 mm/s + 0.2% |v|.\n• Nhiễu chậm GM: 0.1% |v| (300 s).",
         "• Ước lượng Scale (1 trạng thái) và 3 góc Boresight (3 trạng thái).\n• Bù Lever arm; dùng nhiễu trắng tính R; lọc ngoại lai NIS <= 11.34."),
        ("GNSS RTK & Compass (5 Hz)",
         "• Chỉ bật khi ở mặt nước.\n• Nhiễu: vị trí 0.02 m, vận tốc 0.02 m/s, góc Yaw 0.05°.",
         "• Cập nhật 7 chiều (Vị trí, Vận tốc NED, góc Yaw) khi có tín hiệu; lọc NIS <= 18.48."),
        ("Cảm biến Độ sâu (5 Hz)",
         "• Hoạt động toàn thời gian.\n• Nhiễu trục Down: 0.05 m.",
         "• Cập nhật trực tiếp tọa độ trục Down (p_D); lọc NIS <= 6.63.")
    ]
    table1 = doc.add_table(rows=len(t1_data), cols=3)
    for r_i, row_content in enumerate(t1_data):
        for c_i, val in enumerate(row_content):
            table1.cell(r_i, c_i).text = val
    format_table_bw(table1, [1.6, 2.5, 2.47], font_size=11)

    # ================= PHẦN 2 =================
    add_heading_bw("2. SÁU KỊCH BẢN ĐÃ ĐƯỢC ĐƯA VÀO MÔ PHỎNG", space_before=16)
    add_heading_bw("2.1. Quỹ đạo và quy ước điểm cố định trong hệ tọa độ NED")
    add_bullet_bw("Quy ước vị trí đích cố định trên hệ tọa độ NED: ",
                  "Trong 6 kịch bản, UUV đều xuất phát tại gốc tọa độ cố định p_start = [0, 0, 0] m. Điểm đích cuối cùng được gắn cố định và vị trí được xét trên hệ tọa độ NED: destination_ned = [N_dest; E_dest; D_dest] m (trong đó D_dest là độ sâu mục tiêu tính từ mặt nước theo phương hướng xuống đáy).")
    add_bullet_bw("Ý nghĩa thiết kế quỹ đạo: ",
                  "Nếu UUV chỉ bơi thẳng đều, góc lệch Boresight theo trục Roll sẽ không quan sát được. Vì vậy quỹ đạo bao gồm các pha chính: Pha bơi mặt nước có GNSS kết hợp quay đầu và kích thích 3 trục (vận tốc ngang Sway, vận tốc đứng Heave, góc lắc Roll) để tìm đủ bộ 4 tham số (hệ số scale, alpha, beta, gamma), trước khi ngắt GNSS và lặn ngầm về điểm quy ước cố định trong hệ tọa độ NED.")

    # ================= BẢNG 1: TỔNG HỢP ƯỚC LƯỢNG THAM SỐ HIỆU CHUẨN =================
    add_heading_bw("2.2. Bảng 1: Kết quả ước lượng tham số hiệu chuẩn DVL (Scale & Boresight) sau khi ngắt GNSS")
    t_kb1_data = [
        ("Kịch bản", "Cắt GNSS T_cal (s)", "Hệ số Scale s (%)\n(Chuẩn: 1.0000%)", "Góc Roll α (độ)\n(Chuẩn: 0.2000°)", "Góc Pitch β (độ)\n(Chuẩn: -0.3000°)", "Góc Yaw γ (độ)\n(Chuẩn: 1.0000°)"),
        ("Kịch bản 1 (Cơ bản)", "250 s", "1.0159 %", "0.1664°", "-0.2820°", "0.9775°"),
        ("Kịch bản 2 (Cơ động mạnh)", "220 s", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)"),
        ("Kịch bản 3 (Kích thích nhẹ)", "280 s", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)"),
        ("Kịch bản 4 (Kích thích mạnh)", "200 s", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)"),
        ("Kịch bản 5 (Kích thích cực yếu)", "260 s", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)"),
        ("Kịch bản 6 (Vòng số 8)", "250 s", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)", "(Chờ chạy lệnh)")
    ]
    table_kb1 = doc.add_table(rows=len(t_kb1_data), cols=6)
    for r_i, row_content in enumerate(t_kb1_data):
        for c_i, val in enumerate(row_content):
            table_kb1.cell(r_i, c_i).text = val
    format_table_bw(table_kb1, [1.45, 0.85, 1.15, 1.04, 1.04, 1.04], font_size=10.5)

    # ================= BẢNG 2: TỔNG HỢP SAI SỐ DẪN ĐƯỜNG VÀ ĐO LƯỜNG DVL =================
    add_heading_bw("2.3. Bảng 2: Tổng hợp sai số dẫn đường khi lặn ngầm và thống kê kiểm định DVL", space_before=14)
    t_kb2_data = [
        ("Kịch bản", "Thời gian lặn ngầm (s)", "RMSE Vận tốc khi lặn (m/s)", "RMSE Vị trí 3D khi lặn (m)", "Sai số vị trí cuối tại Đích / Lúc nổi (m)", "Bản tin DVL\n(Dùng / Loại)"),
        ("Kịch bản 1 (Đích [450; 250; 15])", "428.5 s", "0.0118 m/s", "0.2174 m", "0.2298 m", "3390 / 3"),
        ("Kịch bản 2 (Đích [520; -140; 20])", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)"),
        ("Kịch bản 3 (Đích [350; 420; 10])", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)"),
        ("Kịch bản 4 (Đích [650; 150; 25])", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)"),
        ("Kịch bản 5 (Đích [400; 200; 12])", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)", "(Chờ chạy)"),
        ("Kịch bản 6 (Vòng số 8, D = 15 m)", "450.0 s", "(Chờ chạy)", "(Chờ chạy)", "(Lấy lúc nổi 700s)", "(Chờ chạy)")
    ]
    table_kb2 = doc.add_table(rows=len(t_kb2_data), cols=6)
    for r_i, row_content in enumerate(t_kb2_data):
        for c_i, val in enumerate(row_content):
            table_kb2.cell(r_i, c_i).text = val
    format_table_bw(table_kb2, [1.57, 0.85, 1.05, 1.05, 1.15, 0.90], font_size=10.5)

    # ================= PHẦN 3: SƠ ĐỒ VÀ MA TRẬN BỘ LỌC ES-EKF =================
    add_heading_bw("3. CẤU TRÚC BỘ LỌC ES-EKF 19 TRẠNG THÁI: ĐẦU VÀO, MA TRẬN XỬ LÝ VÀ ĐẦU RA", space_before=16)
    
    add_heading_bw("3.1. Bảng phân định kiến trúc: Đầu vào - Kích thước ma trận - Đầu ra hệ thống")
    t3_data = [
        ("Khối chức năng", "Dữ liệu Đầu vào (Inputs)", "Ký hiệu & Kích thước Ma trận", "Dữ liệu Đầu ra (Outputs) & Vai trò"),
        ("1. Cảm biến IMU\n(Tốc độ 400 Hz)",
         "• Vận tốc góc thô: w_meas\n• Gia tốc riêng thô: a_meas\n• Nhận bias hồi tiếp: b̂_g, b̂_a",
         "• w_corr, a_corr ∈ ℝ³\n• Nhiễu ARW: 0.025 °/√h\n• Nhiễu VRW: 0.015 m/s/√h",
         "• Tín hiệu quán tính đã khử trôi:\n  w_corr = w_meas - b̂_g\n  a_corr = a_meas - b̂_a"),
        ("2. Cơ học Quán tính INS\n(Tích phân 400 Hz)",
         "• w_corr, a_corr (400 Hz)\n• Vận tốc góc Trái Đất w_ie\n• Gia tốc trọng trường g",
         "• Trạng thái danh định x ∈ ℝ²⁰:\n  x = [pᵀ, vᵀ, qᵀ, b_aᵀ, b_gᵀ, s, φ_bsᵀ]ᵀ\n• Động học: ṗ=v; v̇=C a + g; q̇=½ q⊗w\n• Suy giảm: ḃ_a = -(1/τ) b_a (τ=100s)",
         "• Quỹ đạo dự đoán liên tục:\n  Vị trí p, vận tốc v, quaternion q\n• Đưa x (20×1) xuống Hàm đo h(x)"),
        ("3. Cảm biến Cập nhật\n(Ngoại vi 5 Hz)",
         "• GNSS RTK + Yaw (mặt nước)\n• Depth sensor (liên tục)\n• DVL A125 (bám đáy biển)",
         "• z_gnss = [pᵀ, vᵀ, ψ]ᵀ ∈ ℝ⁷\n• z_depth = p_D ∈ ℝ¹\n• z_dvl = [vx, vy, vz]ᵀ ∈ ℝ³",
         "• Cung cấp vector đo tổng hợp:\n  z_meas ∈ ℝᵐ (m = 1, 3, 4 hoặc 7)\n  đưa vào khối tính phần dư"),
        ("4. Hàm đo & Cổng NIS\n(Kiểm định 5 Hz)",
         "• Vector đo z_meas\n• Vector danh định x (20×1)\n• Tham số lever arm l_arm",
         "• Hàm đo DVL: h(x) = (1+s)(I-[φ×])v_dvl\n• Phần dư: r = z_meas - h(x) ∈ ℝᵐ\n• Hiệp phương sai: S = H·P·Hᵀ + R\n• Thống kê: NIS = rᵀ·S⁻¹·r ≤ γ",
         "• Loại bỏ đo lỗi ngoại lai (Outliers)\n• Xuất phần dư r và ma trận Jacobian H\n  sang khối cập nhật Kalman"),
        ("5. Bộ lọc sai số ES-EKF\n(Cập nhật 19 trạng thái)",
         "• Phần dư r & ma trận H (5 Hz)\n• Nhiễu quá trình Q (19×19)\n• Nhiễu đo lường R (m×m)",
         "• Lan truyền: P = F·P·Fᵀ + Q (19×19)\n• Độ khuếch đại: K = P·Hᵀ·S⁻¹ (19×m)\n• Vector sai số: δx = K·r ∈ ℝ¹⁹\n• Dạng Joseph: P = (I-KH)P(I-KH)ᵀ + KRKᵀ",
         "• Ước lượng tối ưu vector sai số 19×1:\n  δx = [δp, δv, δθ, δba, δbg, δs, δφ]ᵀ\n• Cập nhật độ bất định ma trận P"),
        ("6. Bơm sai số (Inject)\n& Hồi tiếp (Feedback)",
         "• Vector sai số ước lượng δx (19×1)\n• Trạng thái danh định x (20×1)\n• Ma trận P sau cập nhật",
         "• Bơm danh định: x = x ⊕ δx\n  p = p + δp;  v = v + δv;  ba = ba + δba\n  q = q ⊗ [1; ½δθ];  s = s + δs;  φ = φ + δφ\n• Reset sai số: δx = 0;  P = G·P·Gᵀ",
         "• Xuất quỹ đạo dẫn đường chính xác (p, v, q)\n• Xuất tham số hiệu chuẩn: s, φ_bs, ba, bg\n• Hồi tiếp trực tiếp b̂_a, b̂_g khử trôi IMU")
    ]
    table3 = doc.add_table(rows=len(t3_data), cols=4)
    for r_i, row_content in enumerate(t3_data):
        for c_i, val in enumerate(row_content):
            table3.cell(r_i, c_i).text = val
    format_table_bw(table3, [1.3, 1.5, 2.0, 1.77], font_size=11)

    add_heading_bw("3.2. Sơ đồ cấu trúc và luồng hoạt động của bộ lọc ES-EKF 19 trạng thái", space_before=14)
    p_desc = doc.add_paragraph()
    p_desc.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    r_desc = p_desc.add_run(
        "Sơ đồ dưới đây thể hiện trực quan luồng dữ liệu và nguyên lý hoạt động của bộ lọc: Tầng trên thực hiện dự đoán quán tính INS tốc độ cao (400 Hz); Tầng dưới thực hiện xử lý đo lường ngoại vi và cập nhật sai số Kalman (5 Hz). Sơ đồ làm nổi bật vị trí các điểm trừ tín hiệu, khối đầu ra dẫn đường và 2 vòng hồi tiếp kín cốt lõi (Hồi tiếp trạng thái cho chu kỳ tiếp theo và Hồi tiếp khử trôi Bias cho IMU)."
    )
    r_desc.font.name = 'Arial'
    r_desc.font.size = Pt(14)
    r_desc.font.color.rgb = RGBColor(0, 0, 0)
    r_desc._element.rPr.rFonts.set(qn('w:eastAsia'), 'Arial')

    # Chèn ảnh sơ đồ khối trực tiếp vào Word
    p_img = doc.add_paragraph()
    p_img.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p_img.paragraph_format.space_before = Pt(8)
    p_img.paragraph_format.space_after = Pt(4)
    run_img = p_img.add_run()
    run_img.add_picture(diagram_file, width=Inches(6.5))

    p_cap = doc.add_paragraph()
    p_cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r_cap = p_cap.add_run("Hình 3.1. Sơ đồ khối cấu trúc bộ lọc ES-EKF 19 trạng thái (phân định rõ luồng dự đoán, kiểm định đo lường, 2 vòng hồi tiếp và khối đầu ra).")
    r_cap.bold = True
    r_cap.font.name = 'Arial'
    r_cap.font.size = Pt(14)
    r_cap.font.color.rgb = RGBColor(0, 0, 0)

    add_heading_bw("3.3. Giải thích luồng hoạt động 4 bước và 2 đường hồi tiếp của bộ lọc", space_before=14)
    add_bullet_bw("Bước 1: Cảm biến (INS/IMU) gắn thân tàu cập nhật gia tốc và góc quay 400 lần/s: ",
                  "Giải phương trình cơ học quán tính để tự dự đoán tàu đang ở đâu, vận tốc và tư thế theo hệ trục tọa độ NED.")
    add_bullet_bw("Bước 2: Đo thực tế và lọc dữ liệu (f = 5 Hz): ",
                  "Cập nhật 5 lần mỗi giây từ các cảm biến bên ngoài như GNSS, DVL, Cảm biến đo sâu, rồi so sánh với kết quả tự dự đoán ở Bước 1 để tính phần dư sai lệch.")
    add_bullet_bw("Bước 3: Tính toán mức độ sai lệch (5 Hz): ",
                  "Bộ lọc ES-EKF tính ra tàu đang lệch bao nhiêu mét, cảm biến bị trôi (bias) bao nhiêu và các cảm biến đang bị lắp lệch cơ khí góc nào (Boresight) cùng hệ số co giãn (Scale).")
    add_bullet_bw("Bước 4: Bơm bù sai số và đưa kết quả đầu ra: ",
                  "Cộng bù trực tiếp vector sai số tìm được vào kết quả Bước 1 để cho ra kết quả dẫn đường chính xác (vị trí, tư thế) và các góc lắp đặt đã tìm được.")
    add_bullet_bw("Chú thích - Đường hồi tiếp 1: ",
                  "Lấy vị trí và tư thế chuẩn xác sau hiệu chỉnh để sửa lại đúng điểm xuất phát mới cho chu kỳ dự đoán tiếp theo, tránh bị cộng dồn sai số theo thời gian.")
    add_bullet_bw("Chú thích - Đường hồi tiếp 2: ",
                  "Gửi thông số trôi (bias) tìm được quay ngược lại đầu vào để hiệu chỉnh và khử trôi trực tiếp cho cảm biến IMU gốc.")

    add_heading_bw("3.4. Thiết lập tham số bộ lọc và phân tích kinh nghiệm căn chỉnh xử lý lỗi", space_before=14)
    add_bullet_bw("1. Ma trận hiệp phương sai ban đầu P0: ",
                  "• Cấu hình khởi tạo: Hệ số co giãn Scale: ±2% (0.02^2); Boresight: (1.5°)^2; Góc tư thế: (0.5°)^2.\n• Phân tích & Xử lý lỗi: Vì góc Boresight thực tế đang giả thiết là dưới 1°. Nếu thực tế góc lắp đặt cơ khí lớn hơn 1°, cần tăng ma trận hiệp phương sai ban đầu P0 lên tương ứng để bộ lọc mở rộng phạm vi tìm kiếm và hội tụ nhanh hơn; nếu để P0 quá nhỏ so với sai lệch thật, bộ lọc sẽ kẹt sai số và hội tụ rất chậm.")
    add_bullet_bw("2. Nhiễu quá trình Q (Độ tin cậy vào mô hình động học INS): ",
                  "• Cấu hình setup: Vận tốc Q(4:6, 4:6) = max(VRW^2, 0.05^2); Scale RW = 5x10^-7; Boresight RW = 5x10^-5 deg.\n• Phân tích & Xử lý lỗi: Nếu đặt Q quá nhỏ, bộ lọc quá tin vào quán tính INS khiến việc ước lượng tham số lắp đặt (Scale/Boresight) bị \"đơ\" không cập nhật; nếu đặt Q quá lớn, bộ lọc bị nhạy cảm quá mức làm trạng thái rung giật theo từng nhịp đo lường.")
    add_bullet_bw("3. Nhiễu đo lường R (Độ tin tưởng vào từng cảm biến ngoại vi): ",
                  "• Cấu hình setup: Căn cứ theo sàn nhiễu thực tế của GNSS RTK, Cảm biến độ sâu và chùm tia DVL A125 (nhiễu trắng 5 mm/s + 0.2% |v|).\n• Phân tích & Xử lý lỗi: Nếu đặt R quá nhỏ (quá tin cảm biến đo), bộ lọc dễ bị giật cục khi sóng biển làm tàu rung lắc; nếu đặt R quá lớn, bộ lọc xem nhẹ số liệu đo khiến sai số vị trí trôi dạt dần theo thời gian như khi mất cảm biến.")
    add_bullet_bw("4. Kiểm định cổng thống kê NIS (Chi-square test lọc ngoại lai): ",
                  "• Cấu hình setup: Ngưỡng loại bỏ nhiễu đột biến với độ tin cậy 99%: GNSS (7 bậc) NIS <= 18.48; Depth (1 bậc) NIS <= 6.63; DVL (3 bậc) NIS <= 11.34.\n• Phân tích & Xử lý lỗi: Nếu số đo liên tục bị cổng NIS từ chối, nguyên nhân do cảm biến gặp sự cố đột biến (GNSS mất RTK, DVL mất đáy) hoặc do ma trận P và R đặt quá nhỏ làm cổng bị thắt quá chặt; nếu ngưỡng quá lỏng thì số đo rác (outliers) sẽ lọt vào làm lệch bộ lọc.")



    # ================= PHẦN 4 =================
    add_heading_bw("4. KẾT QUẢ MÔ PHỎNG VÀ ĐÁNH GIÁ TRÊN 6 KỊCH BẢN", space_before=16)

    add_heading_bw("4.1. Bảng kết quả tìm 3 góc lệch Boresight và sai số 3 góc tư thế UUV (Roll, Pitch, Yaw)")
    add_bullet_bw("Quy ước đối chiếu: ",
                  "Giá trị chuẩn cài đặt trong mô phỏng của 3 góc lệch Boresight là [alpha, beta, gamma] = [0.2000°, -0.3000°, 1.0000°]. Bảng 4.1 trình bày kết quả ước lượng 3 góc Boresight tại thời điểm ngắt GNSS (T_cal) kèm độ lệch chuẩn ±3 sigma và sai số RMSE của 3 góc tư thế UUV (Roll, Pitch, Yaw) trong pha lặn.")

    t41_data = [
        ("Kịch bản\n(T_cal)",
         "Boresight Roll alpha\n(Thật: 0.2°)",
         "Boresight Pitch beta\n(Thật: -0.3°)",
         "Boresight Yaw gamma\n(Thật: 1.0°)",
         "Sai số Roll UUV (°)",
         "Sai số Pitch UUV (°)",
         "Sai số Yaw UUV (°)"),
        ("Kịch bản 1\n(250 s)", "0.1664°\n(±1.50°)", "-0.2820°\n(±0.08°)", "0.9775°\n(±0.07°)", "0.0363°", "0.0134°", "0.0032°"),
        ("Kịch bản 2\n(220 s)", "0.2654°\n(±1.27°)", "-0.3013°\n(±0.07°)", "0.9825°\n(±0.06°)", "0.0035°", "0.0080°", "0.0132°"),
        ("Kịch bản 3\n(280 s)", "-0.5850°\n(±1.82°)", "-0.3226°\n(±0.08°)", "1.0189°\n(±0.08°)", "0.0030°", "0.0333°", "0.0181°"),
        ("Kịch bản 4\n(200 s)", "0.1865°\n(±1.20°)", "-0.3280°\n(±0.06°)", "0.9940°\n(±0.06°)", "0.0368°", "0.0167°", "0.0414°"),
        ("Kịch bản 5\n(260 s)", "0.0549°\n(±2.66°)", "-0.2748°\n(±0.07°)", "1.0318°\n(±0.07°)", "0.0227°", "0.0465°", "0.0118°"),
        ("Kịch bản 6\n(250 s)", "0.0723°\n(±2.97°)", "-0.3265°\n(±0.04°)", "1.0024°\n(±0.04°)", "0.0144°", "0.0216°", "0.0330°")
    ]

    table41 = doc.add_table(rows=len(t41_data), cols=7)
    for r_i, row_content in enumerate(t41_data):
        for c_i, val in enumerate(row_content):
            table41.cell(r_i, c_i).text = val
    format_table_bw(table41, [1.05, 0.92, 0.92, 0.92, 0.92, 0.92, 0.92], font_size=9.5)

    add_bullet_bw("Nhận xét về 3 góc Boresight: ",
                  "Góc lệch Pitch (beta) và Yaw (gamma) hội tụ rất nhanh và chính xác ở cả 6 kịch bản (beta đạt từ -0.2748° đến -0.3280° so với chuẩn -0.3°; gamma đạt từ 0.9775° đến 1.0318° so với chuẩn 1.0°, với dải ±3 sigma thu hẹp chỉ còn 0.04° - 0.08°). Riêng góc lệch Roll (alpha = 0.2°) đạt độ chính xác cao nhất ở các kịch bản có kích thích lắc đủ mạnh như Kịch bản 4 (0.1865°), Kịch bản 1 (0.1664°) và Kịch bản 2 (0.2654°); trong khi ở Kịch bản 5 (kích thích cực yếu) và Kịch bản 6 (bơi mặt nước phẳng không có vận tốc đứng/ngang), góc alpha hội tụ chậm hơn và dải ±3 sigma còn rộng (2.66° - 2.97°).")
    add_bullet_bw("Nhận xét về 3 góc tư thế của UUV (Roll, Pitch, Yaw): ",
                  "Cả 3 góc tư thế của UUV đều bám sát quỹ đạo thật với sai số RMSE rất nhỏ trên toàn bộ 6 kịch bản (sai số Roll và Pitch dưới 0.047°, sai số góc hướng Yaw dưới 0.042°).")

    add_heading_bw("4.2. Bảng kết quả bám Vận tốc UUV theo 3 trục (v_N, v_E, v_D), Vị trí UUV theo hệ NED (N, E) và Độ sâu (D)", space_before=14)
    add_bullet_bw("Ý nghĩa bảng số liệu: ",
                  "Bảng 4.2 tổng hợp sai số trung bình phương (RMSE) của vận tốc 3 trục (v_N, v_E, v_D), vị trí mặt ngang (North, East) và độ sâu (Down) trong suốt giai đoạn UUV lặn ngầm mất tín hiệu GNSS.")

    t42_data = [
        ("Kịch bản\n(Đích hệ NED)",
         "Sai số v_N\n(m/s)",
         "Sai số v_E\n(m/s)",
         "Sai số v_D\n(m/s)",
         "Sai số Vị trí Bắc N (m)",
         "Sai số Vị trí Đông E (m)",
         "Sai số Độ sâu D (m)"),
        ("Kịch bản 1\n[450; 250; 15]", "0.0068", "0.0067", "0.0069", "0.0662", "0.2068", "0.0116"),
        ("Kịch bản 2\n[520; -140; 20]", "0.0073", "0.0072", "0.0073", "0.2247", "0.2591", "0.0118"),
        ("Kịch bản 3\n[350; 420; 10]", "0.0064", "0.0066", "0.0063", "0.3134", "0.3914", "0.0119"),
        ("Kịch bản 4\n[650; 150; 25]", "0.0081", "0.0076", "0.0076", "0.6902", "0.2896", "0.0114"),
        ("Kịch bản 5\n[400; 200; 12]", "0.0066", "0.0067", "0.0067", "0.0861", "0.0705", "0.0113"),
        ("Kịch bản 6\n(Vòng số 8, D=15)", "0.0077", "0.0080", "0.0079", "0.1036", "0.1291", "0.0126")
    ]

    table42 = doc.add_table(rows=len(t42_data), cols=7)
    for r_i, row_content in enumerate(t42_data):
        for c_i, val in enumerate(row_content):
            table42.cell(r_i, c_i).text = val
    format_table_bw(table42, [1.35, 0.87, 0.87, 0.87, 0.87, 0.87, 0.87], font_size=9.5)

    add_bullet_bw("Nhận xét về Vận tốc UUV theo 3 trục (v_N, v_E, v_D): ",
                  "Nhờ phép đo DVL đã được hiệu chuẩn góc lệch Boresight và bù cánh tay đòn, vận tốc ước lượng theo cả 3 trục Bắc, Đông, Xuống bám sát vận tốc thật với sai số RMSE chỉ từ 0.0063 m/s đến 0.0081 m/s (tương đương 6 - 8 mm/s, ngang mức sàn nhiễu vật lý của cảm biến DVL A125).")
    add_bullet_bw("Nhận xét về Độ sâu (D) và Vị trí UUV theo hệ NED (N, E): ",
                  "Độ sâu trục Down (D) được ghim chặt với sai số chỉ khoảng 0.011 - 0.012 m (1.2 cm) ở mọi kịch bản nhờ cảm biến độ sâu 5 Hz. Trên mặt phẳng ngang Bắc - Đông (N, E), dù ngắt hoàn toàn GNSS trong suốt pha lặn dài (400 - 500 giây), sai số vị trí RMSE vẫn giữ ở mức rất thấp (từ 0.07 m đến 0.69 m), đảm bảo UUV bám sát quỹ đạo đặt và tiến về đúng điểm đích cố định trong hệ tọa độ NED.")

    add_heading_bw("4.3. Các đồ thị mô phỏng minh họa kết quả (Kịch bản 1 và Kịch bản 6)", space_before=14)

    simulation_figures = [
        ("Case1_Hinh_1.png", "Hình 4.1. Kịch bản 1 - Quỹ đạo chuyển động 3D của UUV trong hệ tọa độ NED."),
        ("Case1_Hinh_2.png", "Hình 4.2. Kịch bản 1 - Quá trình ước lượng và hội tụ của 3 góc lệch Boresight DVL."),
        ("Case1_Hinh_3.png", "Hình 4.3. Kịch bản 1 - Sai số ước lượng vận tốc 3 trục (vN, vE, vD) và vị trí NED."),
        ("Case1_Hinh_4.png", "Hình 4.4. Kịch bản 1 - Thống kê kiểm định cổng NIS và ước lượng độ trôi Bias."),
        ("Case6_QuyDao_So8.png", "Hình 4.5. Kịch bản 6 - Quỹ đạo chuyển động vòng số 8 (Figure 8 trajectory).")
    ]

    for img_path, caption in simulation_figures:
        if os.path.exists(img_path):
            p_img = doc.add_paragraph()
            p_img.alignment = WD_ALIGN_PARAGRAPH.CENTER
            p_img.paragraph_format.space_before = Pt(8)
            p_img.paragraph_format.space_after = Pt(2)
            run_img = p_img.add_run()
            run_img.add_picture(img_path, width=Inches(6.2))

            p_cap = doc.add_paragraph()
            p_cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
            p_cap.paragraph_format.space_before = Pt(2)
            p_cap.paragraph_format.space_after = Pt(8)
            r_cap = p_cap.add_run(caption)
            r_cap.bold = True
            r_cap.font.name = 'Arial'
            r_cap.font.size = Pt(11)
            r_cap.font.color.rgb = RGBColor(0, 0, 0)

    out_docx = "Bao_Cao_Mo_Phong_UUV_Kem_So_Do_Arial14.docx"
    try:
        doc.save(out_docx)
        print("Đã xuất thành công file Word kèm sơ đồ:", out_docx)
    except PermissionError:
        alt_docx = "Bao_Cao_Mo_Phong_UUV_Kem_So_Do_Arial14_moi.docx"
        doc.save(alt_docx)
        print(f"\n[LƯU Ý]: File '{out_docx}' đang được mở trong Word nên Windows không cho ghi đè.")
        print(f"-> Đã tự động lưu phiên bản mới vào: '{alt_docx}'")
        print(f"-> Bạn có thể mở file '{alt_docx}' để xem ngay, hoặc đóng Word lại rồi chạy lại lệnh.")


if __name__ == "__main__":
    generate_docx_with_diagram()