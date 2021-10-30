#ifndef  _KALMAN_FILTER_H
#define  _KALMAN_FILTER_H
#ifdef __cplusplus
extern "C"
{
#endif
typedef struct {
    double x;  /* state */
    double A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    double C;  /* z(n)=C*x(n)+w(n),w(n)~N(0,r)   */
    double q;  /* process(predict) noise convariance */
    double r;  /* measure noise convariance */
    double p;  /* estimated error convariance */
    double gain;
} kalman_state;

extern void kalman_init(kalman_state *state, double init_x, double init_p);
extern double kalman_filter(kalman_state *state, double z_measure);
#ifdef __cplusplus
}
#endif
#endif  /*_KALMAN_FILTER_H*/

