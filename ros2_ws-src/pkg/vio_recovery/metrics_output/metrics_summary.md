# Metriche di Odometria e Mapping

Le traiettorie e le mappe VIO sono allineate alla GT con traslazione e rotazione yaw iniziale.

| Esperimento | Durata (s) | ATE RMSE (m) | ATE max (m) | Errore 3D finale (m) | Map Precision | Map Recall (Cov) | Map F1 |
|-------------|-----------|-------------|------------|---------------------|---------------|------------------|--------|
| disable_recovery | 252.4 | 0.3918 | 1.3025 | 0.5792 | 0.0433 | 0.0039 | 0.0071 |
| drop_only | 487.8 | 0.9853 | 1.8413 | 1.7035 | 0.0355 | 0.0036 | 0.0066 |
| prova1_350_750_potinc | 469.1 | 0.1529 | 0.3548 | 0.353 | 0.013 | 0.0023 | 0.0039 |
| prova2_250_500_potinc | 470.4 | 0.1705 | 0.3286 | 0.275 | 0.0129 | 0.0017 | 0.0031 |
| prova3_250_500_inc | 328.2 | 0.3014 | 0.7424 | 0.7423 | 0.0438 | 0.0064 | 0.0112 |
| prova4_350_750_inc | 475.6 | 0.1704 | 0.2568 | 0.2471 | 0.048 | 0.0072 | 0.0126 |
| prova5_350_750_inc_velup | 372.8 | 0.4507 | 0.8798 | 0.8798 | 0.0124 | 0.0019 | 0.0033 |
| swipe_only | 511.1 | 0.2051 | 0.4862 | 0.3488 | 0.0831 | 0.0116 | 0.0204 |
