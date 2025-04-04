
/**
*
* @verbatim
Reference:
  J. Boehm, R. Heinkelmann, H. Schuh, Short Note: A Global Model of Pressure
  and Temperature for Geodetic Applications, Journal of Geodesy,
  doi:10.1007/s00190-007-0135-3, 2007.

  input data
  ----------
  dmjd: modified julian date
  dlat: ellipsoidal latitude in radians
  dlon: longitude in radians
  dhgt: ellipsoidal height in m

  output data
  -----------
  pres: pressure in hPa
  temp: temperature in Celsius
  undu: Geoid undulation in m (from a 9x9 EGM based model)

    History
    2011-11-28  JD: created

  @endverbatim
* Copyright (c) 2018 G-Nut Software s.r.o. (software@gnutsoftware.com)
*
* @file       ggpt.h
* @brief      Purpose: Global Pressure Temperature model
* @author     JD
* @version    1.0.0
* @date       2011-11-28
*
*/

#ifndef GGPT_H
#define GGPT_H

#include <iostream>
#include <string.h>
#include <math.h>
#include "gexport/ExportLibGnut.h"

using namespace std;

namespace gnut
{

    /** @brief class for t_gpt. */
    class LibGnut_LIBRARY_EXPORT t_gpt
    {

    public:
        /** @brief default constructor. */
        t_gpt(){};

        /** @brief default destructor. */
        ~t_gpt(){};

        /**
        *@brief       GPT empirical model v1
        * dlat, dlon --> RADIANS !
        */
        int gpt_v1(double dmjd, double dlat, double dlon, double dhgt,
                   double &pres, double &temp, double &undu);

    protected:
        static double a_geoid[55];
        static double b_geoid[55];
        static double ap_mean[55];
        static double bp_mean[55];
        static double ap_amp[55];
        static double bp_amp[55];
        static double at_mean[55];
        static double bt_mean[55];
        static double at_amp[55];
        static double bt_amp[55];

    private:
    };


    /** @brief class for t_gpt. */
    class LibGnut_LIBRARY_EXPORT t_gpt3
    {

    public:
        /** @brief default constructor. */
        t_gpt3() {};

        /** @brief default destructor. */
        ~t_gpt3() {};

        double _ah_gpt3 = 0.0;
        double _aw_gpt3 = 0.0;
        double _p_gpt3 = 0.0;
        double _t_gpt3 = 0.0;
        double _tm_gpt3 = 0.0;
        double _e_gpt3 = 0.0;
        double _la_gpt3 = 0.0;;

        bool getGPT3Data(string GPT3path, double mjd, double lat, double lon, double h_ell, int it
            , double* p, double* T, double* dT, double* Tm, double* e, double* ah, double* aw
            , double* la, double* undu, double* Gn_h, double* Ge_h, double* Gn_w, double* Ge_w);

        void gCopyArray(double* destination, const double* source, int n);

        void gArrayMinus(double* destination, const double* source, double* minus, int n);

        void split_string(bool lnoempty, char* start, char* end, const char* separator, int* nword, char** word);

        int modified_julday(int iday, int imonth, int iyear);

        void mjd2doy(int jd, int* iyear, int* idoy);

        void GPT3_map(double ah, double aw, double mjd, double lat, double lon, double h_ell, double zd, double* dmap, double* wmap);

        double asknewet_gpt3(double e, double Tm, double lambda);

    };
}

#endif
