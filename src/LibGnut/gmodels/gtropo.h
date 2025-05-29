
/**
*
* @verbatim
    History
    2011-01-10 /JD: created

  @endverbatim
* Copyright (c) 2018 G-Nut Software s.r.o. (software@gnutsoftware.com)
*
* @file       gtropo.h
* @brief      Purpose: implements troposphere model class
*.
* @author     JD
* @version    1.0.0
* @date       2011-01-10
*
*/

#ifndef GTROPO_H
#define GTROPO_H

#include "gutils/gtime.h"
#include "gutils/gtriple.h"
#include "gprod/gprodcrd.h"
#include "gmodels/ggpt.h"
#include "gexport/ExportLibGnut.h"

using namespace std;

namespace gnut
{

    /** @brief class for t_gtropo. */
    class LibGnut_LIBRARY_EXPORT t_gtropo
    {
    public:
        //t_gtropo(string site);

        /** @brief default constructor. */
        t_gtropo();

        /** @brief default destructor. */
        virtual ~t_gtropo();

        /** @brief get ZHD. */
        virtual double getZHD(const t_gtriple &ell, const t_gtime &epo, string str_gpt = ""); // ! Radians: Ell[0] and Ell[1]

        /** @brief get ZWD. */
        virtual double getZWD(const t_gtriple &ell, const t_gtime &epo, string str_gpt = ""); // ! Radians: Ell[0] and Ell[1]

        double _zhd = 0.0;
        double _zwd = 0.0;
        double _map_wet = 0.0;
        double _map_dty = 0.0;
        double _ele = 0.0;


        double h_abs_bott[3] = { 0.00126851,0.00269736,0.0558965 };
        double w_abs_bott[3] = { 0.00053815,0.00134966,0.0398130 };

        double h_abs_top[3] = { 0.00125249,0.0026974 ,0.05589515 };
        double w_abs_top[3] = { 0.00048605,0.00134979,0.03981546 };

        //metro ZTD
        vector<double> _sod_metro = { 6600.0,8400.0,10200.0,12000.0,13800.0,15600.0,17400.0,
                                    19200.0,21000.0,22800.0,24600.0,26400.0,28200.0,30000.0,31800.0,33600.0,35400.0 };
        vector<double> _zhd_metro_bott = { 2.285,2.286,2.285,2.285,2.284,2.283,2.282,2.281,2.280,2.279,2.278,2.278,2.278,2.278,2.278,2.278,2.278 };
        vector<double> _zwd_metro_bott = { 0.224,0.233,0.240,0.241,0.240,0.241,0.243,0.243,0.245,0.247,0.248,0.251,0.251,0.254,0.253,0.254,0.254 };
        vector<double> _zhd_metro_top = { 2.079,2.079,2.079,2.079,2.078,2.078,2.077,2.075,2.075,2.074,2.073,2.073,2.073,2.073,2.073,2.073,2.073 };
        vector<double> _zwd_metro_top = { 0.208,0.212,0.214,0.215,0.217,0.219,0.217,0.218,0.217,0.216,0.219,0.219,0.219,0.220,0.219,0.219,0.219 };


        //ERA5 ZTD
        vector<double> _sod_era5 = { 3600.0,7200.0,10800.0,14400.0,18000.0,21600.0,25200.0,28800.0,32400.0,36000.0 };
        vector<double> _zhd_era5_bott = { 2.285,2.286,2.286,2.284,2.283,2.281,2.280,2.279,2.279,2.280 };
        vector<double> _zwd_era5_bott = { 0.250,0.248,0.251,0.258,0.264,0.269,0.271,0.272,0.269,0.273 };
        //vector<double> _zhd_era5_top = { 2.079,2.080,2.080,2.079,2.078,2.076,2.076,2.075,2.075,2.076 };
        //vector<double> _zwd_era5_top = { 0.168,0.165,0.167,0.173,0.178,0.182,0.184,0.183,0.181,0.185 };
        //vector<double> _zhd_era5_zx3_top = { 2.0359,2.0368,2.0368,2.0350,2.0341,2.0324,2.0315,2.0306,2.0306,2.0315 };
        //vector<double> _zwd_era5_zx3_top = { 0.2227,0.2210,0.2236,0.2299,0.2352,0.2397,0.2415,0.2424,0.2397,0.2432 };
        vector<double> _zhd_era5_top = { 2.0359,2.0368,2.0368,2.0350,2.0341,2.0324,2.0315,2.0306,2.0306,2.0315 };
        vector<double> _zwd_era5_top = { 0.2227,0.2210,0.2236,0.2299,0.2352,0.2397,0.2415,0.2424,0.2397,0.2432 };

    public:
        t_gpt _gpt; ///< gpt

        t_gpt3 _gpt3;
    };

    /** @brief class for t_saast derive from t_gtropo. */
    class LibGnut_LIBRARY_EXPORT t_saast : public t_gtropo
    {
    public:


    public:
        /** @brief default constructor. */
        t_saast() {}

        /** @brief default destructor. */
        ~t_saast() {}

        double getZTD(const t_gtriple& ell, const t_gtime& epo);

        /** @brief get STD/ZHD/ZWD. */
        virtual double getSTD(const double &ele, const double &hel);     // ! Radians: elevation
        virtual double getZHD(const t_gtriple &ell, const t_gtime &epo, string str_gpt = ""); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const t_gtriple &ell, const t_gtime &epo, string str_gpt = ""); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for t_davis derive from t_gtropo. */
    class LibGnut_LIBRARY_EXPORT t_davis : public t_gtropo
    {
    public:
        /** @brief default constructor. */
        t_davis() {}

        /** @brief default destructor. */
        ~t_davis() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for t_hopf derive from t_gtropo. */
    class LibGnut_LIBRARY_EXPORT t_hopf : public t_gtropo
    {
    public:
        /** @brief default constructor. */
        t_hopf() {}

        /** @brief default destructor. */
        ~t_hopf() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for t_baby derive from t_gtropo. */
    class LibGnut_LIBRARY_EXPORT t_baby : public t_gtropo
    {
    public:
        /** @brief default constructor. */
        t_baby() {}

        /** @brief default destructor. */
        ~t_baby() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for t_chao derive from t_gtropo. */
    class LibGnut_LIBRARY_EXPORT t_chao : public t_gtropo
    {
    public:
        /** @brief default constructor. */
        t_chao() {}

        /** @brief default destructor. */
        ~t_chao() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
    };

    /** @brief class for t_ifad derive from t_gtropo. */
    class LibGnut_LIBRARY_EXPORT t_ifad : public t_gtropo
    {
    public:
        /** @brief default constructor. */
        t_ifad() {}

        /** @brief default destructor. */
        ~t_ifad() {}

        /** @brief get ZHD/ZWD. */
        virtual double getZHD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
        virtual double getZWD(const t_gtriple &ele, const t_gtime &epo); // ! Radians: Ell[0] and Ell[1]
    };

} // namespace

#endif
