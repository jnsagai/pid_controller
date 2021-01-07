#ifndef TWIDDLE_H
#define TWIDDLE_H
#include <vector>

class TWIDDLE
{
public:
    /**
   * Constructor
   */
    TWIDDLE(double tolerance);

    /**
   * Destructor.
   */
    virtual ~TWIDDLE();

    void Run();

    void SetError(const double cte);

    std::vector<double> GetParams();

private:

    enum States
    {
        START,
        FIRST_RUN,
        INCREMENT,
        DECREMENT,
        DONE
    };

    double curr_error;
    double best_error;
    double tolerance;

    States curr_state;
    unsigned int param_index;
    
    std::vector<double> params;
    std::vector<double> d_params;

    

    void ChangeParamIndex();
};

#endif // TWIDDLE_H