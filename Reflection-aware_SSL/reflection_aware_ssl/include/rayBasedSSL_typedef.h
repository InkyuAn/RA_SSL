#ifndef RAYBASED_SOUNDLOCALIZATION_TYPEDEF_H
#define RAYBASED_SOUNDLOCALIZATION_TYPEDEF_H

#include <iostream>
#include <cmath>

namespace raybased_soundlocalization{
    class soundAttenuation{
    public:
        soundAttenuation() :
            temperature(0.0),
            humidity(0.0),
            atmosphericAbsorption_F_ro(0.0),
            atmosphericAbsorption_F_rn(0.0),
            atmosphericAbsorption_a(0.0),
            atmosphericAbsorption_b(0.0),
            atmosphericAbsorption_c(0.0)
        {

        }
        ~soundAttenuation()
        {

        }

        void init(double temperature_, double humidity_)
        {

            temperature = temperature_ + 273.15;
            humidity = humidity_ * std::pow(10.0, -6.8346*std::pow(273.16/temperature, 1.261) + 4.6151);
            atmosphericAbsorption_F_ro = 24 + 40400 * humidity * (0.02 + humidity) / (0.391+humidity);
            atmosphericAbsorption_F_rn = std::pow(temperature/293.15, -0.5) * (9 + 280*humidity*std::exp(-4.17*(std::pow(temperature/293.15, -1.0/3.0)-1)));
            atmosphericAbsorption_a = 0.01599 * std::pow(10.0, -8.0) * std::pow(temperature/293.15, 0.5);
            atmosphericAbsorption_b = 0.110797 * std::exp(-2239.1/temperature);
            atmosphericAbsorption_c = 0.928092 * std::exp(-3352.0/temperature);

            /*std::cout << "T, h: " << T << " " << h << std::endl;
            std::cout << "F_ro, F_rn: " << ps_atmosphericAbsorption_F_ro << " " << ps_atmosphericAbsorption_F_rn << std::endl;
            std::cout << "a, b, c: " << ps_atmosphericAbsorption_a << " " << ps_atmosphericAbsorption_b << " " << ps_atmosphericAbsorption_c << std::endl;*/
        }

        double getFro() {return atmosphericAbsorption_F_ro;}
        double getFrn() {return atmosphericAbsorption_F_rn;}
        double getA() {return atmosphericAbsorption_a;}
        double getB() {return atmosphericAbsorption_b;}
        double getC() {return atmosphericAbsorption_c;}

    private:
        double temperature;
        double humidity;

        double atmosphericAbsorption_F_ro;
        double atmosphericAbsorption_F_rn;
        double atmosphericAbsorption_a;
        double atmosphericAbsorption_b;
        double atmosphericAbsorption_c;

    };
}

#endif
