#define PIN_IR A0
#define POINTS 7 // 0 ~ 30까지 포인트 수 (현재 5 간격)

void setup() {
  Serial.begin(1000000);

  unsigned int filtered;
  float sensor[POINTS];
  float dist[POINTS];

  // 데이터 수집
  for (int i=0; i<=30; i+=5) {
    while (Serial.available() == 0)
      ;
    Serial.read();

    filtered = ir_sensor_filtered(20, 0.5, 0);
    Serial.print("DIST: "); Serial.print(i);
    Serial.print(", FLT:"); Serial.println(filtered);
    
    sensor[i/5] = (float)filtered;
    dist[i/5] = (float)i;
  }

  // 최소 제곱법 (2차)
  double Sx0 = POINTS; // Σ1
  double Sx1 = 0, Sx2 = 0, Sx3 = 0, Sx4 = 0;
  double Sy = 0, Sxy = 0, Sx2y = 0;

  for (int i = 0; i < POINTS; i++) {
    double x = sensor[i];
    double y = dist[i];
    double x2 = x * x;
    double x3 = x2 * x;
    double x4 = x2 * x2;

    Sx1 += x;
    Sx2 += x2;
    Sx3 += x3;
    Sx4 += x4;

    Sy  += y;
    Sxy += x * y;
    Sx2y += x2 * y;
  }

  float A[3][4] = {
    { (float)Sx0, (float)Sx1, (float)Sx2, (float)Sy   },
    { (float)Sx1, (float)Sx2, (float)Sx3, (float)Sxy  },
    { (float)Sx2, (float)Sx3, (float)Sx4, (float)Sx2y }
  };

  for (int i = 0; i < 3; i++) {
    float pivot = A[i][i];
    if (fabs(pivot) < 1e-6) continue; 

    // 대각을 1로 만들기
    for (int j = i; j < 4; j++) {
      A[i][j] /= pivot;
    }

    // 다른 행에서 제거
    for (int r = 0; r < 3; r++) {
      if (r == i) continue;
      float factor = A[r][i];
      for (int c = i; c < 4; c++) {
        A[r][c] -= factor * A[i][c];
      }
    } 
  }

  float a0 = A[0][3];
  float a1 = A[1][3];
  float a2 = A[2][3];

  Serial.print(a0, 6);
  if (a1 >= 0) Serial.print(" + "); else Serial.print(" - ");
  Serial.print(fabs(a1), 6);
  Serial.print(" * x");

  if (a2 >= 0) Serial.print(" + "); else Serial.print(" - ");
  Serial.print(fabs(a2), 6);
  Serial.println(" * x * x");
}

void loop() {
  
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
  // n: number of measurement repetition
  // position: the percentile of the sample to be taken (0.0 <= position <= 1.0)
  // verbose: 0 - normal operation, 1 - observing the internal procedures, and 2 - calculating elapsed time.
  // Example 1: ir_sensor_filtered(n, 0.5, 0) => return the median value among the n measured samples.
  // Example 2: ir_sensor_filtered(n, 0.0, 0) => return the smallest value among the n measured samples.
  // Example 3: ir_sensor_filtered(n, 1.0, 0) => return the largest value among the n measured samples.

  // The output of Sharp infrared sensor includes lots of spiky noise.
  // To eliminate such a spike, ir_sensor_filtered() performs the following two steps:
  // Step 1. Repeat measurement n times and collect n * position smallest samples, where 0 <= postion <= 1.
  // Step 2. Return the position'th sample after sorting the collected samples.

  // returns 0, if any error occurs

  unsigned int *ir_val, ret_val;
  unsigned int start_time;
 
  if (verbose >= 2)
    start_time = millis(); 

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;
    
  if (position == 1.0)
    position = 0.999;

  if (verbose == 1) {
    Serial.print("n: "); Serial.print(n);
    Serial.print(", position: "); Serial.print(position); 
    Serial.print(", ret_idx: ");  Serial.println((unsigned int)(n * position)); 
  }

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  if (verbose == 1)
    Serial.print("IR:");
  
  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
    if (verbose == 1) {
        Serial.print(" ");
        Serial.print(ir_val[i]);
    }
  }

  if (verbose == 1)
    Serial.print  ("  => ");

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];

  if (verbose == 1) {
    for (int i = 0; i < n; i++) {
        Serial.print(" ");
        Serial.print(ir_val[i]);
    }
    Serial.print(" :: ");
    Serial.println(ret_val);
  }
  free(ir_val);

  if (verbose >= 2) {
    Serial.print("Elapsed time: "); Serial.print(millis() - start_time); Serial.println("ms");
  }
  
  return ret_val;
}
