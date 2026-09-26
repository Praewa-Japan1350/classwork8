# 🤖 RoboMaster EP Autonomous SLAM & Maze Exploration (ClassWork 8)

ระบบสำรวจเขาวงกตและสร้างแผนที่อัตโนมัติ (Autonomous SLAM & Exploration) สำหรับหุ่นยนต์ **DJI RoboMaster EP** ในสนามตารางกริดจำลองขนาด 4×5 ช่อง (ขนาดช่องละ 60×60 ซม.) พร้อมฟังก์ชันตรวจจับแนวกำแพงโฟมด้วยเซนเซอร์ ToF, การจัดตำแหน่งกึ่งกลางช่อง (Recentering), การคำนวณเส้นทางด้วย BFS และการประเมินความถูกต้องของแผนที่ (Map Accuracy & Coverage)

---

## 📌 คุณสมบัติเด่น (Features)

- **Autonomous Exploration (BFS):** ใช้อัลกอริทึม Breadth-First Search ค้นหาเส้นทางที่สั้นที่สุดไปยังช่องที่ยังไม่ได้สำรวจ (Unvisited Cells) จนครบทั้งเขาวงกต
- **Gimbal 4-Direction Scanning:** หมุนกิมบอลควบคุมเซนเซอร์ ToF สแกนรอบทิศทาง 4 ทิศ (North, East, South, West) โดยไม่ต้องหมุนตัวถังหุ่นยนต์ ช่วยลด Wheel Slip และ Drift
- **In-Cell Recentering:** ปรับจูนตำแหน่งหุ่นยนต์ให้อยู่กึ่งกลางช่องด้วยล้อ Mecanum ก่อนเคลื่อนที่ก้าวต่อไป
- **Real-Time GUI Dashboard:** หน้าต่างแสดงผลแบบเรียลไทม์ผ่าน Tkinter แสดงตำแหน่งหุ่นยนต์, ทิศทางหัวกิมบอล, แนวกำแพงที่ตรวจพบ และ Log ข้อมูลสถานะ
- **Comprehensive Map Evaluation:** คำนวณค่า **Coverage (%)** และ **Map Accuracy (%)** เทียบกับ Ground Truth พร้อมสร้างรายงานผลและบันทึกพิกัดกำแพง
- **High-Resolution Map Export:** บันทึกภาพแผนที่เส้นทางเดินและแนวกำแพงกราฟิกระดับ Cyber Dark HUD (`robot_trajectory.png`) อัตโนมัติ
- **Simulation & Re-Evaluation Mode:** รองรับโหมดจำลอง (`--sim`) โดยไม่ต้องเชื่อมต่อหุ่นจริง และโหมดประเมินผลย้อนหลัง (`--eval`) จากไฟล์ CSV เดิม

---

## 📁 โครงสร้างโปรเจกต์ (Project Structure)

```text
classwork8/
├── code/
│   ├── config.py          # การตั้งค่าระบบ (ขนาดกริด, ความเร็ว, ค่า Threshold, Ground Truth)
│   ├── dashboard.py       # หน้าต่าง GUI Dashboard แสดงผล Live Map และสถานะ
│   ├── evaluation.py      # คำนวณ Accuracy, Coverage และวาดภาพ robot_trajectory.png
│   ├── navigation.py      # อัลกอริทึม BFS หาเส้นทาง และตรวจสอบแนวกำแพง
│   ├── robot_control.py   # การควบคุมหุ่นยนต์, เซนเซอร์ ToF, กิมบอล, ล้อ Mecanum
│   ├── slam.py            # ลูปหลักของการสำรวจ (Exploration Engine)
│   └── state.py           # ตัวแปรสถานะส่วนกลาง (State Management)
├── exploration_log.csv    # บันทึกประวัติการเดินแต่ละก้าว (ตำแหน่ง, มุม, ค่า ToF, เวลา)
├── map_evaluation_report.txt # รายงานสรุปเปอร์เซ็นต์ Coverage และ Map Accuracy
├── robot_trajectory.png   # ภาพแผนที่สรุปเส้นทางการเดินและแนวกำแพง
├── run.py                 # สคริปต์หลักสำหรับสั่งรันโปรแกรม
├── walls_data.csv         # พิกัดแนวกำแพงแนวนอน (H) และแนวตั้ง (V) ที่ตรวจพบ
└── .gitignore             # ไฟล์ยกเว้นการติดตามของ Git
```

---

## ⚙️ ความต้องการของระบบ (Requirements)

- **Python 3.8+**
- แพ็กเกจที่จำเป็น:
  - `matplotlib`
  - `robomaster` (กรณีใช้งานกับหุ่นยนต์จริง DJI RoboMaster EP)
  - `tkinter` (มาพร้อมกับ Python มาตรฐานบน Windows)

ติดตั้งแพ็กเกจด้วยคำสั่ง:
```bash
pip install matplotlib
# สำหรับเชื่อมต่อหุ่นยนต์จริง:
pip install robomaster
```

---

## 🚀 วิธีการใช้งาน (Usage)

### 1. รันโหมดจำลอง (Simulation Mode)
ทดสอบระบบอัลกอริทึมและ GUI โดยไม่ต้องเชื่อมต่อกับหุ่นยนต์จริง:
```bash
python run.py --sim
```

### 2. รันกับหุ่นยนต์จริง (Robot Mode)
เชื่อมต่อ Wi-Fi ของคอมพิวเตอร์เข้ากับ Wi-Fi Access Point ของ RoboMaster EP แล้วสั่งรัน:
```bash
python run.py
```

### 3. รันแบบกำหนดจุดเริ่มต้น หรือไม่เปิด Dashboard
```bash
# กำหนดจุดเริ่มต้นที่พิกัด (1, 1) หันหน้าไปทางทิศเหนือ
python run.py --sim --start-x 1 --start-y 1 --start-heading NORTH

# รันแบบไม่เปิดหน้าต่าง GUI (Headless mode)
python run.py --sim --no-dashboard
```

### 4. ประเมินผลแผนที่และวาดภาพใหม่ (Re-evaluation Mode)
นำข้อมูลจากไฟล์ `walls_data.csv` และ `exploration_log.csv` ที่เคยบันทึกไว้ มาคำนวณและอัปเดตภาพ `robot_trajectory.png` รวมถึงรายงานผลใหม่ โดยไม่ต้องนำหุ่นไปวิ่งใหม่:
```bash
python run.py --eval
```
หรือ:
```bash
python code/evaluation.py
```

---

## 📊 รายละเอียดผลลัพธ์ (Outputs)

เมื่อการสำรวจเสร็จสิ้น ระบบจะสร้างไฟล์ผลลัพธ์ออกมาที่โฟลเดอร์หลักโดยอัตโนมัติ:

1. **`robot_trajectory.png`**:
   - ภาพแผนที่ขนาดความละเอียดสูง (300 DPI) ธีม Tactical Cyber Red & Blue
   - แสดงตารางกริด (4×5), แนวกำแพงโฟมสีแดง (Foam Walls), ช่องที่สำรวจแล้ว (Explored Cells), จุดเริ่มต้น (START), จุดสิ้นสุด (END) และลูกศรทิศทางการเดิน
2. **`map_evaluation_report.txt`**:
   - สรุปตัวเลขค่าชี้วัดความแม่นยำ:
     - **Coverage (%)**: ร้อยละของจำนวนช่องที่หุ่นยนต์สามารถเข้าถึงและสำรวจได้สำเร็จ
     - **Map Accuracy (%)**: ความแม่นยำของการทายกำแพงรอบช่องแต่ละช่องเทียบกับ Ground Truth
     - สรุปจำนวนแนวกำแพงที่ตรวจพบเทียบกับ Ground Truth
3. **`exploration_log.csv`**:
   - ข้อมูล Telemetry แบบทีละก้าว ได้แก่ `step`, `timestamp`, `grid_x`, `grid_y`, `x_m`, `y_m`, `heading`, `tof_mm`
4. **`walls_data.csv`**:
   - พิกัดแนวกำแพงโฟมแนวนอน (H) และแนวตั้ง (V) สำหรับนำไปวิเคราะห์ต่อในโปรแกรมอื่น เช่น MATLAB หรือ Python Data Analysis

---

## 🔧 การปรับแต่งค่าใน `code/config.py`

| พารามิเตอร์ | ค่าเริ่มต้น | คำอธิบาย |
|---|---|---|
| `GRID_W`, `GRID_H` | `4, 5` | ขนาดตารางกริด (กว้าง 4 ช่อง, สูง 5 ช่อง) |
| `GRID_SIZE_M` | `0.60` | ขนาดของแต่ละช่อง (0.60 เมตร / 60 ซม.) |
| `SPEED` | `0.38` | ความเร็วในการเคลื่อนที่ตรง (m/s) |
| `ROT_SPEED` | `50` | ความเร็วในการหมุนตัวถัง (deg/s) |
| `WALL_THRESHOLD_MM` | `550` | ระยะ ToF สูงสุดที่จัดว่าเป็นกำแพงกั้นขอบช่อง (mm) |
| `GROUND_TRUTH_H_WALLS` | Set ของพิกัด | แนวกำแพงแนวนอนจริงตามแบบสนามอาจารย์ |
| `GROUND_TRUTH_V_WALLS` | Set ของพิกัด | แนวกำแพงแนวตั้งจริงตามแบบสนามอาจารย์ |
