# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Hồ Đăng Phúc
- MSSV: 2A202602796
- Email: phucdhoangh@gmail.com
- Link repo (fork): https://github.com/hdphuc127/K4-L3-DAY23-HoDangPhuc-2A202602796
- Commit hash nộp (`git rev-parse HEAD`): 775e2f8d8ada12d7ba0b7b95617439302e53c9d0

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: 0.9701, 0.7004, 519 / 16 / 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1503 m, 502, 11.3436, 0, 239, 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1359 m, 502, 9.2668, 0, 239, 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: hai mode có **cùng** `matches = 502`, `ghost_track_frames = 0`, `missed_gt_frames = 239` và `mean_confirmed_tracks = 2.52`. Điều này khớp với track-then-fuse: camera không tạo, xác nhận hay xoá track, nên tập track confirmed và các cặp ghép với GT giống hệt nhau. Vì so trên cùng 502 cặp, RMSE so sánh được trực tiếp: fused giảm `sum_sq_err` từ 11.34 xuống 9.27 và RMSE từ 0.150 xuống 0.136 m (−0.0145 m, ≤ +0.05 m nên camera không làm xấu tracking). `precision_track = 502/(502+0) = 1.00`, `coverage = 502/519 = 0.967`. 239 frame-miss chủ yếu do detector bỏ sót (`det_fn = 222`) và track mới chưa đủ hit để confirmed, không phải do camera. Lưu ý camera ở đây là tâm hộp 2D GT có nhiễu seeded, nên mức cải thiện không đại diện cho một camera detector thật.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?

   Lidar: `z` 3x1 là vị trí (x, y, z) mét trong hệ sensor, `h(x)` tuyến tính (phép biến đổi cứng), `H` = phần xoay của `veh_to_sens` cộng 3 cột vận tốc bằng 0, `R = diag(0.1²)` m². Camera: `z` 2x1 là pixel (u, v), `h(x)` phi tuyến (pinhole `u = c_i − f_i·y/x`, `v = c_j − f_j·z/x`), nên `H` là Jacobian tính tại trạng thái dự đoán (EKF thật sự), `R = diag(5²)` px². Camera không đo được độ sâu, nên chủ yếu hiệu chỉnh hướng ngang/dọc.

2. Vì sao cần gating Mahalanobis trước khi gán?

   Khoảng cách Mahalanobis `γᵀS⁻¹γ` chuẩn hóa residual theo độ bất định (S = HPHᵀ + R) và đưa khoảng cách mét với pixel về cùng thang đo. Cổng χ² (ngưỡng 99.5%, bậc tự do = `dim_meas`) loại các cặp gần như không thể đúng trước khi gán greedy, để một đo xa/nhiễu/clutter không kéo track lệch và không bị dùng để update sai.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.

   Track-then-fuse: chỉ có một danh sách track; mỗi frame `ekf_predict` một lần → `associate_and_update(..., lidar)` (update + score/init/delete) → `associate_and_update(..., camera)` chỉ update EKF. Trên log: ở mode `fused`, số `confirmed` mỗi frame được quyết định bởi lidar; camera không sinh track mới (mode lidar và fused cùng detection `det_tp/fp/fn`), khác biệt chỉ thể hiện ở `sum_sq_err`/RMSE.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?

   Innovation camera có bias hệ thống (trung bình khác 0, cùng hướng qua nhiều frame/track) thay vì nhiễu trắng quanh 0; Mahalanobis tăng, nhiều cặp bị cổng χ² loại; những cặp lọt cổng kéo vị trí track lệch nên RMSE fused > RMSE lidar.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.

   Khi `meas_list` rỗng thì không suy ra sensor từ `meas.sensor` được, nhưng vẫn phải gọi `manage_tracks` để lidar trừ score các track trong FOV bị miss và xóa track hết score; nếu bỏ qua, track ma tồn tại mãi. Lidar là cảm biến 3D đầy đủ và là nguồn detection thật sự của pipeline; camera chỉ có 2D, FOV hẹp (FRONT) và trong lab là tâm hộp GT có nhiễu, nên nếu camera được cộng/trừ score thì track ngoài FOV camera sẽ bị phạt oan và tính tồn tại của track sẽ phụ thuộc GT. Vì vậy camera chỉ tinh chỉnh trạng thái.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.

   Khởi tạo `score = 1/6`, `initialized`. Mỗi frame lidar: hit `+1/6` (tối đa 1), miss trong FOV lidar `−1/6`. `score > 0.8` → `confirmed`; track confirmed giữ nguyên trạng thái khi miss. Xóa nếu `Pxx` hoặc `Pyy` > `max_P = 9` m², hoặc confirmed và `score < 0.6`, hoặc chưa confirmed và `score ≤ 0`. Camera không bao giờ kích hoạt xóa.


## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code
- Dùng cho phần nào (hàm, câu hỏi, debug): implement Part E–H, debug lỗi OpenMP trên Windows, soạn nháp câu trả lời
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` (128 passed), đối chiếu công thức với docs/HUONG_DAN_KY_THUAT.md

## Checklist nộp

- [ ] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [ ] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [ ] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [ ] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [ ] Đã điền đủ file này, gồm khai báo AI
- [ ] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [ ] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
