# Lưu Trữ & Quản Lý Kết Quả Thực Nghiệm Bằng SQLite

Hệ thống hỗ trợ tự động ghi nhận toàn bộ kết quả thực thi của solver vào cơ sở dữ liệu **SQLite** (`results/results.db`) song song với file CSV và text truyền thống.

---

## 1. Cấu Trúc Bảng (Database Schema)

Cơ sở dữ liệu bao gồm bảng chính `experiments` với đầy đủ các thuộc tính của bài toán, cấu hình giải thuật và thống kê nghiệm:

```sql
CREATE TABLE IF NOT EXISTS experiments (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
    instance TEXT NOT NULL,
    n_customers INTEGER,
    n_drones INTEGER,
    swap_time_min REAL,
    battery_capacity REAL,
    drone_cap REAL,
    truck_speed REAL,
    drone_speed REAL,
    assignment_mode TEXT,
    slave TEXT,
    truck_mode TEXT,
    branching TEXT,
    feasible INTEGER,
    best_upper_bound REAL,
    best_lower_bound REAL,
    final_gap REAL,
    final_gap_percent REAL,
    truck_value REAL,
    drone_value REAL,
    final_makespan REAL,
    initial_solution REAL,
    time_to_best_solution_sec REAL,
    runtime_sec REAL,
    num_truck_customers INTEGER,
    num_drone_customers INTEGER,
    truck_customers TEXT,
    drone_customers TEXT,
    truck_route TEXT,
    drone_routes TEXT,
    gurobi_status INTEGER,
    gurobi_master_obj REAL,
    gurobi_final_master_lb REAL,
    best_tracked_master_lb REAL,
    gurobi_master_gap REAL,
    valid_incumbent_updates INTEGER,
    mipnode_cuts INTEGER,
    local_search INTEGER
);
```

### Chỉ mục và View tóm tắt:
* **Indexes**: Đã đánh chỉ mục trên `instance`, `feasible`, `slave`, `assignment_mode` giúp truy vấn cực nhanh khi lượng dữ liệu lớn.
* **View `v_summary`**: Bảng nhìn nhanh gồm các trường: `id`, `timestamp`, `instance`, `n_customers`, `n_drones`, `assignment_mode`, `slave`, `feasible`, `UB`, `LB`, `gap_pct`, `runtime_sec`, `gurobi_status`.

---

## 2. Tự Động Lưu Trữ Từ C++ Solver

Solver tự động ghi kết quả vào SQLite database tại đường dẫn mặc định `results/results.db`:

```bash
# Sử dụng mặc định (results/results.db)
./build/truck_drone_solver data/instances/legacy_toy_6.txt --assignment explicit --slave milp --truck sec

# Chỉ định file SQLite database khác:
./build/truck_drone_solver data/instances/legacy_toy_6.txt --db results/my_experiments.db
```

Nếu không muốn ghi vào SQLite, truyền `--db ""` để tắt.

---

## 3. Công Cụ Quản Lý Dữ Liệu `scripts/manage_db.py`

Script Python `scripts/manage_db.py` cung cấp các lệnh quản trị tiện ích:

### 3.1. Xem danh sách các lần chạy gần nhất
```bash
python3 scripts/manage_db.py list --limit 20
```

### 3.2. Xem chi tiết toàn bộ thông tin một lần chạy (theo ID)
```bash
python3 scripts/manage_db.py show 96
```

### 3.3. Import các kết quả cũ từ file CSV vào Database
```bash
# Import một file CSV đơn lẻ:
python3 scripts/manage_db.py import-csv results/csv/result.csv

# Quét và import toàn bộ file CSV trong thư mục results:
python3 scripts/manage_db.py import-all --dir results
```

### 3.4. Xuất dữ liệu từ SQLite ra file CSV
```bash
python3 scripts/manage_db.py export-csv results/exported_all_runs.csv
```

### 3.5. Thực hiện câu lệnh SQL tùy ý
```bash
python3 scripts/manage_db.py sql "SELECT instance, slave, UB, LB, gap_pct, runtime_sec FROM v_summary WHERE feasible = 1 ORDER BY runtime_sec ASC LIMIT 10;"
```
