class EnvDetector
{
    public:
        EnvDetector(double k1 = 0.0, double k2 = 0.0, double Ts = 1.0, double x1 = 0.0, double x2 = 0.0) : 
            k1(k1), k2(k2), Ts(Ts), x1(x1), x2(x2) 
        {}

        double GetSmp(double u) {
            auto y{ (x1 > x2) ? x1 : x2 };
            auto epsilon{ u - y };
            x1 += k1*Ts*epsilon;
            x2 += k2*Ts*epsilon;
            
            return y;
        }

        

    private:
        double k1;
        double k2;
        double x1;
        double x2;
        double Ts;

};
