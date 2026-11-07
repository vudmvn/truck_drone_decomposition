# Model-to-code mapping

| LaTeX symbol | Meaning | Code location |
|---|---|---|
| \(t_i\) | customer `i` served by truck | `MasterModelBuilder` |
| \(a_{ik}\) | customer `i` assigned to drone `k` | `MasterModelBuilder`, `FullSolution::per_drone_customers` |
| \(x_{ij}\) | truck arc from `i` to `j` | `MasterModelBuilder`, `RouteUtils` |
| \(z_T\) | truck route duration | `FullSolution::truck_value` |
| \(z_k\) | drone `k` lower-bound/master estimate | `MasterCuts`, future exact master |
| \(z_D\) | max drone makespan | `FullSolution::drone_value` |
| \(z\) | total makespan | `FullSolution::objective` |
| \(S_k\) | fixed customer set of drone `k` | `FullSolution::per_drone_customers[k]` |
| \(D_k^*(S_k)\) | optimal slave value for drone `k` | `SingleDroneOrderingSlave::Solve` |
| subset cut | conditional lower-bound cut | `MasterCuts` |

