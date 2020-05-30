class EnvDetector
{
    public:
        EnvDetector(double k1 = 0, double k2 = 0, double Ts = 1) : k1(k1), k2(k2), x1(0), x2(0), Ts(Ts) {}

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
