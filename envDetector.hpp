class EnvDetector
{
    public:
        EnvDetector() : k1(.0), k2(.0), x1(.0), x2(.0), Ts(1.0) {}
        EnvDetector(double k1, double k2, double Ts) : k1(k1), k2(k2), x1(.0), x2(.0), Ts(Ts) {}

        double GetSmp(double u) {
            auto y{ (x1 > x2) ? x1 : x2 };
            auto ϵ{ u - y };
            x1 += k1*Ts*ϵ;
            x2 += k2*Ts*ϵ;
            
            return y;
        }

        

    private:
        double k1;
        double k2;
        double x1;
        double x2;
        double Ts;

};
