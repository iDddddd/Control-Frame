
void ekf_init(void * ekf, int n, int m);


int ekf_update_time(void* v, double T1);


int ekf_predict_step(void* v);

/**
  * Runs one step of EKF prediction and update. Your code should first build a model, setting
  * the contents of <tt>ekf.fx</tt>, <tt>ekf.F</tt>, <tt>ekf.hx</tt>, and <tt>ekf.H</tt> to appropriate values.
  * @param ekf pointer to structure EKF
  * @param z array of measurement (observation) values
  * @return 0 on success, 1 on failure caused by non-positive-definite matrix.
  */
int ekf_update_step(void * v, double * z);


int ekf_partial_update_step(void* v, double* z, const int* indexes, const int num_of_indexes);


void ekf_set_R(void* v, double R[25]);
