# Lipschitz-type LBBD cuts trong PDSTSP với battery swapping

Giả thiết: Mỗi drone thực hiện các sortie độc lập `depot -> i -> depot`; 
không có time windows, không có ràng buộc pin dùng chung giữa các drone;
`e_i <= B` với mỗi khách khả thi bằng drone; các drone đồng nhất, `tau >= 0`.

Ký hiệu `r_i`: thời gian bay khứ hồi của khách `i`; `tau`: thời gian đổi pin;
`phi(D)`: makespan nhỏ nhất để `K` drone phục vụ toàn bộ `D`;
`phi_k(S)`: thời gian tối thiểu khi **một** drone phục vụ `S`.

## 1. LBBD từ certified lower bound

Với tập tham chiếu `S`, giả sử slave chứng minh được `L(S) <= phi(S)`.
Khi loại bỏ khách `i` khỏi `S`, độ giảm makespan tối đa là `r_i + tau`.
Thật vậy, từ bất kỳ lịch khả thi cho `S\{i}` ta luôn có thể bổ sung `i`
vào cuối một drone đang hoạt động, đổi pin nếu cần, với thời gian tăng
không quá `r_i+tau`. Với tập rỗng, chọn một drone và không cần đổi pin.

Lần lượt thêm lại các khách bị thiếu cho thấy:

`phi(D) >= L(S) - sum_{i in S\D} (r_i+tau)`.

Đồng thời `phi(D)>=0`; do đó có thể cắt mỗi hệ số tại `L(S)`:

```
z_D + sum_{i in S} min(r_i + tau, L(S)) * (1-y_i) >= L(S).
```

Trong chế độ `explicit`, thay `z_D` bằng `z_k` và `y_i` bằng `a_{ik}`:

```
z_k + sum_{i in S} min(r_i + tau, L_k(S)) * (1-a_ik) >= L_k(S).
```

**Điều kiện:** `L(S)` phải là certified LB. Objective của nghiệm khả thi
chưa chứng minh tối ưu **không được** dùng làm RHS của LBBD cut.

## 2. Fractional subset workload cuts

Với một tập `S`, đặt:

`R_K(S)=max(0,ceil(sum_{i in S} e_i/B)-K)`.

Khi đó `K*z_D >= F_K(D)` với:

`F_K(S)=sum_{i in S} r_i + tau*R_K(S)`.

Việc xóa một khách `i` có `e_i<=B` làm giảm `ceil(sum e/B)` tối đa 1,
cho nên `F_K(S)-F_K(S\{i}) <= r_i+tau`. Vì vậy:

```
K*z_D + sum_{i in S} min(r_i+tau,F_K(S))*(1-y_i) >= F_K(S).
```

Với một drone, `K=1` và dùng `z_k`, `a_ik` tương ứng.

## 3. Energy cover cuts

Tương tự, số chu kỳ pin tối thiểu theo tổng năng lượng giảm tối đa 1
khi xóa một khách drone có `e_i<=B`. Do đó:

```
Q + sum_{i in S}(1-y_i) >= R_K(S).
q_k + sum_{i in S}(1-a_ik) >= R_1(S).
```

Hệ số **1** trong hai ràng buộc này là hệ số removal-Lipschitz của
**số lần đổi pin**, không phải hệ số thời gian `r_i+tau`.

## 4. Static pair workload cuts

Nếu hai khách `i,j` cùng thuộc một drone, số lần đổi pin ít nhất là
`h_ij = 1[e_i+e_j>B]`, và bound là `F_ij=r_i+r_j+tau*h_ij`.

```
z_k + min(F_ij,r_i+tau*h_ij)*(1-a_ik)
    + min(F_ij,r_j+tau*h_ij)*(1-a_jk) >= F_ij.
```

## 5. Những Big-M không thể thay theo cách này

- MTZ truck `ordT[j] >= ordT[i]+1 - M*(1-x_ij)` mã hóa thứ tự tuyến xe.
- Battery propagation ở one-drone MILP cần indicator constraints hoặc
  giá trị Big-M được chứng minh chặt, không dùng `r_i+tau` được.
- Các ràng buộc tuyến đường/logic khác không biểu diễn lower-bound của một
  hàm tập hợp cũng không thể thay bằng Lipschitz LBBD.

Thay đổi này chỉ áp dụng cho nhóm **conditional slave lower-bound cuts**.
