
/* ----------------------------------------------------------------------
 * G-Nut - GNSS software development library
 * 
  (c) 2018 G-Nut Software s.r.o. (software@gnutsoftware.com)
  This file is part of the G-Nut C++ library.
 
-*/

#include <cmath>
#include <iomanip>

#include "gmodels/gtropo.h"
#include "gutils/gconst.h"
#include "gutils/gtypeconv.h"
#include "gutils/gsysconv.h"

using namespace std;

namespace gnut
{

    t_gtropo::t_gtropo() 
    {
        //vector<double> _sod_metro = { 6600.0,8400.0,10200.0,12000.0,13800.0,15600.0,17400.0,
        //                    19200.0,21000.0,22800.0,24600.0,26400.0,28200.0,30000.0,31800.0,33600.0 };
        //vector<double> _zhd_metro = { 2.285,2.286,2.285,2.285,2.284,2.283,2.282,2.281,2.280,
        //                            2.279,2.278,2.278,2.278,2.278,2.278,2.278 };
        //vector<double> _zwd_metro = { 0.224,0.233,0.240,0.241,0.240,0.241,0.243,0.243,0.245
        //                            ,0.247,0.248,0.251,0.251,0.254,0.253,0.254 };

        //vector<double> _sod_era5 = { 3600.0,7200.0,10800.0,14400.0,18000.0,21600.0,25200.0,28800.0,32400.0,36000.0 };
        //vector<double> _zhd_era5_bott = { 2.285,2.286,2.286,2.284,2.283,2.281,2.280,2.279,2.279,2.280, };
        //vector<double> _zwd_era5_bott = { 0.250,0.248,0.251,0.258,0.264,0.269,0.271,0.272,0.269,0.273 };
        //vector<double> _zhd_era5_top = { 2.079,2.080,2.080,2.079,2.078,2.076,2.076,2.075,2.075,2.076 };
        //vector<double> _zhd_era5_top = { 0.168,0.165,0.167,0.173,0.178,0.182,0.184,0.183,0.181,0.185 };
    }

    t_gtropo::~t_gtropo()
    {
    }

    double t_gtropo::getZHD(const t_gtriple &ell, const t_gtime &epo) // ell v RADIANECH !! TREBA SJEDNOTIT
    {

        double pp = 1013.25 * pow(1.0 - 2.26e-5 * ell[2], 5.225);
        double res = (0.002277 * pp) / (1.0 - 0.00266 * cos(2.0 * ell[0]) - 0.00000028 * ell[2]);

        cerr << "t_gtropo:getZHD not available data " << epo.str_ymdhms() << ", using default: 2.3m \n";

        return res;
    }

    double t_gtropo::getZWD(const t_gtriple &ell, const t_gtime &epo) // ell v RADIANECH !! TREBA SJEDNOTIT
    {

        return 0.0; // NWM_UNKNOWN;
    }

    double t_saast::getZTD(const t_gtriple& ell, const t_gtime& epo)
    {

        return 0.0;
    }

    double t_saast::getSTD(const double &ele, const double &height)
    {

        double pp = 1013.25 * pow(1.0 - 2.26e-5 * height, 5.225);
        double TT = 18.0 - height * 0.0065 + 273.15;
        double hh = 50.0 * exp(-6.396e-4 * height);
        double ee = hh / 100.0 * exp(-37.2465 + 0.213166 * TT - 0.000256908 * TT * TT);

        double h_km = height / 1000.0;

        if (h_km < 0.0)
            h_km = 0.0;
        if (h_km > 5.0)
            h_km = 5.0;
        int ii = int(h_km + 1);
        double href = ii - 1;

        double bCor[6];
        bCor[0] = 1.156;
        bCor[1] = 1.006;
        bCor[2] = 0.874;
        bCor[3] = 0.757;
        bCor[4] = 0.654;
        bCor[5] = 0.563;

        double BB = bCor[ii - 1] + (bCor[ii] - bCor[ii - 1]) * (h_km - href);

        double zen = G_PI / 2.0 - ele;
        double delay = (0.002277 / cos(zen)) * (pp + ((1255.0 / TT) + 0.05) * ee - BB * (tan(zen) * tan(zen)));
        return delay;
    }

    double t_saast::getZHD(const t_gtriple &Ell, const t_gtime &epoch)
    {
        if (!WPD_TEST)
        {
			double P, T, N;

			_gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

			double delay = (0.002277 * P) /
				(1.0 - 0.00266 * cos(2.0 * Ell[0]) - 0.00000028 * Ell[2]);
            return delay;
        }
        else
        {
            //GPT3
            //double p_gpt3 = 0.0, t_gpt3 = 0.0, tm_gpt3 = 0.0, e_gpt3 = 0.0, ah_gpt3 = 0.0, aw_gpt3 = 0.0, la_gpt3 = 0.0;
            //_gpt3.getGPT3Data("gpt3_1.grd", epoch.mjd(), Ell[0], Ell[1], Ell[2], 0
            //    , &p_gpt3, &t_gpt3, nullptr, &tm_gpt3, &e_gpt3, &ah_gpt3, &aw_gpt3, &la_gpt3, nullptr
            //    , nullptr, nullptr, nullptr, nullptr);

            double delay = (0.002277 * _gpt3._p_gpt3) /
                (1.0 - 0.00266 * cos(2.0 * Ell[0]) - 0.00000028 * Ell[2]);
            this->_zhd = delay;
            return delay;

            //if (Ell[2] < 500)//BOTT
            //{
            //    for (int i = 0; i < _sod_era5.size() - 1; i++)
            //    {
            //        if (epoch.sod() >= _sod_era5[i] && epoch.sod() <= _sod_era5[i + 1])
            //        {
            //            return _zhd_era5_bott[i];
            //        }
            //    }
            //}
            //else if (Ell[2] > 500)//TOP
            //{
            //    for (int i = 0; i < _sod_era5.size() - 1; i++)
            //    {
            //        if (epoch.sod() >= _sod_era5[i] && epoch.sod() <= _sod_era5[i + 1])
            //        {
            //            return _zhd_era5_top[i];
            //        }
            //    }
            //}

        }     
    }

    double t_saast::getZWD(const t_gtriple &Ell, const t_gtime &epoch)
    {
        if(!WPD_TEST)
        {
            double P, T, N;
            _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

            //add need test
            double hh = 0.6;
            double e = hh * 6.11 * pow(10.0, (7.5 * T / (T + 237.3)));

            T += 273.15;

            double delay = (0.0022768 * e * (1255.0 / T + 0.05)) / (1.0 - 0.00266 * cos(2.0 * Ell[0]) - 0.00000028 * Ell[2]);
            return delay;
        }
        else
        {
            //if (this->_zwd > 1e-9)
			//GPT3
			_gpt3._p_gpt3 = 0.0;
			_gpt3._t_gpt3 = 0.0;
			_gpt3._tm_gpt3 = 0.0;
			_gpt3._e_gpt3 = 0.0;
			_gpt3._la_gpt3 = 0.0;
			_gpt3._ah_gpt3 = 0.0;
			_gpt3._aw_gpt3 = 0.0;
			_gpt3.getGPT3Data("gpt3_1.grd", epoch.mjd(), Ell[0], Ell[1], Ell[2], 0
				, &_gpt3._p_gpt3, &_gpt3._t_gpt3, nullptr, &_gpt3._tm_gpt3, &_gpt3._e_gpt3, &_gpt3._ah_gpt3, &_gpt3._aw_gpt3, &_gpt3._la_gpt3, nullptr
				, nullptr, nullptr, nullptr, nullptr);
			double delay = _gpt3.asknewet_gpt3(_gpt3._e_gpt3, _gpt3._tm_gpt3, _gpt3._la_gpt3);

			_gpt3.GPT3_map(_gpt3._ah_gpt3, _gpt3._aw_gpt3, epoch.mjd(), Ell[0], Ell[1], Ell[2]
				, G_PI / 2.0 - this->_ele, &this->_map_dty, &this->_map_wet);
			this->_zwd = delay;
			return delay;


            //if (Ell[2] < 500)//BOTT
            //{
            //    double ah = this->h_abs_bott[0];
            //    double bh = this->h_abs_bott[0];
            //    double ch = this->h_abs_bott[0];
            //    double aw = this->w_abs_bott[0];
            //    double bw = this->w_abs_bott[0];
            //    double cw = this->w_abs_bott[0];
            //    double el = this->ele;

            //    this->_map_dty = (1 + (ah / (1 + bh / (1 + ch)))) / (sin(el) + (ah / (sin(el) + bh / (sin(el) + ch))));
            //    this->_map_wet = (1 + (aw / (1 + bw / (1 + cw)))) / (sin(el) + (aw / (sin(el) + bw / (sin(el) + cw))));

            //    for (int i = 0; i < _sod_era5.size()-1; i++)
            //    {
            //        if (epoch.sod() >= _sod_era5[i] && epoch.sod() <= _sod_era5[i + 1])
            //        {
            //            return _zwd_era5_bott[i];
            //        }
            //    }
            //}
            //else if(Ell[2] > 500)//TOP
            //{
            //    double ah = this->h_abs_top[0];
            //    double bh = this->h_abs_top[0];
            //    double ch = this->h_abs_top[0];
            //    double aw = this->w_abs_top[0];
            //    double bw = this->w_abs_top[0];
            //    double cw = this->w_abs_top[0];
            //    double el = this->ele;

            //    this->_map_dty = (1 + (ah / (1 + bh / (1 + ch)))) / (sin(el) + (ah / (sin(el) + bh / (sin(el) + ch))));
            //    this->_map_wet = (1 + (aw / (1 + bw / (1 + cw)))) / (sin(el) + (aw / (sin(el) + bw / (sin(el) + cw))));
            //    for (int i = 0; i < _sod_era5.size() - 1; i++)
            //    {
            //        if (epoch.sod() >= _sod_era5[i] && epoch.sod() <= _sod_era5[i + 1])
            //        {
            //            return _zwd_era5_top[i];
            //        }
            //    }
            //}
        }

    }

    double t_davis::getZHD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        double P, T, N;
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        double delay = (0.0022768 * P) / (1.0 - 0.0026 * cos(2.0 * Ell[0]) - 0.00000028 * Ell[2]);

        return delay;
    }

    double t_davis::getZWD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        // not implemented yet
        return 0.0;
    }

    double t_hopf::getZHD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        double P, T, N;

        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        double delay = (1e-06 / 5.0) * (77.64 * (P / T)) * (40136.0 + 148.72 * T);
        return delay;
    }

    double t_hopf::getZWD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        double P, T, N;
        double hh = 50.0 * exp(-6.396e-4 * Ell[2]);
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        double e = hh / 100.0 * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

        double delay = (1.e-06 / 5.0) * ((-12.96) * (e / T) + (3.718 * 1.e05) * (e / (T * T))) * 11000.0;
        return delay;
    }

    double t_baby::getZHD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        double P, T, N;
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        T += 273.15; // [K]

        double gs = 9.81; // [ms^2] surface gravity !!!
        double rs = A_WGS + Ell[2];
        double sigma = Eps / T;
        double mu = gs / (Rd * Eps) * (1.0 - (2.0 / (rs * sigma)));

        double delay = (0.022277 * P / gs) * (1.0 + (2.0 / (rs * sigma * (mu + 1.0))));
        return delay;
    }

    double t_baby::getZWD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        // not implemeted yet
        return 0.0;
    }

    double t_chao::getZHD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        // not implemeted yet
        return 0.0;
    }

    double t_chao::getZWD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        double P, T, N, alpha;
        double hh = 50.0 * exp(-6.396e-4 * Ell[2]);
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        T += 273.15;
        alpha = 0.0065; // model is not very sensitive to the temperature lapse rate

        double e = hh / 100.0 * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

        double delay = 4.70 * 100.0 * (pow(e, 1.23) / (T * T)) + 1.71 * 1.e6 * (pow(e, 1.46) / (T * T * T)) * alpha;
        return delay;
    }

    double t_ifad::getZHD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        // not implemeted yet
        return 0.0;
    }

    // ---------
    double t_ifad::getZWD(const t_gtriple &Ell, const t_gtime &epoch)
    {

        double P, T, N;
        double hh = 50.0 * exp(-6.396e-4 * Ell[2]);
        _gpt.gpt_v1(epoch.mjd(), Ell[0], Ell[1], Ell[2], P, T, N);

        T += 273.15;

        double e = hh / 100.0 * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

        double delay = 0.00554 - 0.0880 * 1.e-4 * (P - 1000.0) + 0.272 * 1.e-4 * e + 2.771 * (e / T);
        return delay;
    }

} // namespace
