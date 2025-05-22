
/* ----------------------------------------------------------------------
 * G-Nut - GNSS software development library
 * 
  (c) 2018 G-Nut Software s.r.o. (software@gnutsoftware.com)
 
  This file is part of the G-Nut C++ library.
 
-*/

#include "gmodels/ggpt.h"
#include "gutils/gconst.h"

namespace gnut
{

    double t_gpt::a_geoid[55] = {
        -5.6195e-001, -6.0794e-002, -2.0125e-001, -6.4180e-002, -3.6997e-002,
        +1.0098e+001, +1.6436e+001, +1.4065e+001, +1.9881e+000, +6.4414e-001,
        -4.7482e+000, -3.2290e+000, +5.0652e-001, +3.8279e-001, -2.6646e-002,
        +1.7224e+000, -2.7970e-001, +6.8177e-001, -9.6658e-002, -1.5113e-002,
        +2.9206e-003, -3.4621e+000, -3.8198e-001, +3.2306e-002, +6.9915e-003,
        -2.3068e-003, -1.3548e-003, +4.7324e-006, +2.3527e+000, +1.2985e+000,
        +2.1232e-001, +2.2571e-002, -3.7855e-003, +2.9449e-005, -1.6265e-004,
        +1.1711e-007, +1.6732e+000, +1.9858e-001, +2.3975e-002, -9.0013e-004,
        -2.2475e-003, -3.3095e-005, -1.2040e-005, +2.2010e-006, -1.0083e-006,
        +8.6297e-001, +5.8231e-001, +2.0545e-002, -7.8110e-003, -1.4085e-004,
        -8.8459e-006, +5.7256e-006, -1.5068e-006, +4.0095e-007, -2.4185e-008};

    double t_gpt::b_geoid[55] = {
        +0.0000e+000, +0.0000e+000, -6.5993e-002, +0.0000e+000, +6.5364e-002,
        -5.8320e+000, +0.0000e+000, +1.6961e+000, -1.3557e+000, +1.2694e+000,
        +0.0000e+000, -2.9310e+000, +9.4805e-001, -7.6243e-002, +4.1076e-002,
        +0.0000e+000, -5.1808e-001, -3.4583e-001, -4.3632e-002, +2.2101e-003,
        -1.0663e-002, +0.0000e+000, +1.0927e-001, -2.9463e-001, +1.4371e-003,
        -1.1452e-002, -2.8156e-003, -3.5330e-004, +0.0000e+000, +4.4049e-001,
        +5.5653e-002, -2.0396e-002, -1.7312e-003, +3.5805e-005, +7.2682e-005,
        +2.2535e-006, +0.0000e+000, +1.9502e-002, +2.7919e-002, -8.1812e-003,
        +4.4540e-004, +8.8663e-005, +5.5596e-005, +2.4826e-006, +1.0279e-006,
        +0.0000e+000, +6.0529e-002, -3.5824e-002, -5.1367e-003, +3.0119e-005,
        -2.9911e-005, +1.9844e-005, -1.2349e-006, -7.6756e-009, +5.0100e-008};

    double t_gpt::ap_mean[55] = {
        +1.0108e+003, +8.4886e+000, +1.4799e+000, -1.3897e+001, +3.7516e-003,
        -1.4936e-001, +1.2232e+001, -7.6615e-001, -6.7699e-002, +8.1002e-003,
        -1.5874e+001, +3.6614e-001, -6.7807e-002, -3.6309e-003, +5.9966e-004,
        +4.8163e+000, -3.7363e-001, -7.2071e-002, +1.9998e-003, -6.2385e-004,
        -3.7916e-004, +4.7609e+000, -3.9534e-001, +8.6667e-003, +1.1569e-002,
        +1.1441e-003, -1.4193e-004, -8.5723e-005, +6.5008e-001, -5.0889e-001,
        -1.5754e-002, -2.8305e-003, +5.7458e-004, +3.2577e-005, -9.6052e-006,
        -2.7974e-006, +1.3530e+000, -2.7271e-001, -3.0276e-004, +3.6286e-003,
        -2.0398e-004, +1.5846e-005, -7.7787e-006, +1.1210e-006, +9.9020e-008,
        +5.5046e-001, -2.7312e-001, +3.2532e-003, -2.4277e-003, +1.1596e-004,
        +2.6421e-007, -1.3263e-006, +2.7322e-007, +1.4058e-007, +4.9414e-009};

    double t_gpt::bp_mean[55] = {
        +0.0000e+000, +0.0000e+000, -1.2878e+000, +0.0000e+000, +7.0444e-001,
        +3.3222e-001, +0.0000e+000, -2.9636e-001, +7.2248e-003, +7.9655e-003,
        +0.0000e+000, +1.0854e+000, +1.1145e-002, -3.6513e-002, +3.1527e-003,
        +0.0000e+000, -4.8434e-001, +5.2023e-002, -1.3091e-002, +1.8515e-003,
        +1.5422e-004, +0.0000e+000, +6.8298e-001, +2.5261e-003, -9.9703e-004,
        -1.0829e-003, +1.7688e-004, -3.1418e-005, +0.0000e+000, -3.7018e-001,
        +4.3234e-002, +7.2559e-003, +3.1516e-004, +2.0024e-005, -8.0581e-006,
        -2.3653e-006, +0.0000e+000, +1.0298e-001, -1.5086e-002, +5.6186e-003,
        +3.2613e-005, +4.0567e-005, -1.3925e-006, -3.6219e-007, -2.0176e-008,
        +0.0000e+000, -1.8364e-001, +1.8508e-002, +7.5016e-004, -9.6139e-005,
        -3.1995e-006, +1.3868e-007, -1.9486e-007, +3.0165e-010, -6.4376e-010};

    double t_gpt::ap_amp[55] = {
        -1.0444e-001, +1.6618e-001, -6.3974e-002, +1.0922e+000, +5.7472e-001,
        -3.0277e-001, -3.5087e+000, +7.1264e-003, -1.4030e-001, +3.7050e-002,
        +4.0208e-001, -3.0431e-001, -1.3292e-001, +4.6746e-003, -1.5902e-004,
        +2.8624e+000, -3.9315e-001, -6.4371e-002, +1.6444e-002, -2.3403e-003,
        +4.2127e-005, +1.9945e+000, -6.0907e-001, -3.5386e-002, -1.0910e-003,
        -1.2799e-004, +4.0970e-005, +2.2131e-005, -5.3292e-001, -2.9765e-001,
        -3.2877e-002, +1.7691e-003, +5.9692e-005, +3.1725e-005, +2.0741e-005,
        -3.7622e-007, +2.6372e+000, -3.1165e-001, +1.6439e-002, +2.1633e-004,
        +1.7485e-004, +2.1587e-005, +6.1064e-006, -1.3755e-008, -7.8748e-008,
        -5.9152e-001, -1.7676e-001, +8.1807e-003, +1.0445e-003, +2.3432e-004,
        +9.3421e-006, +2.8104e-006, -1.5788e-007, -3.0648e-008, +2.6421e-010};

    double t_gpt::bp_amp[55] = {
        +0.0000e+000, +0.0000e+000, +9.3340e-001, +0.0000e+000, +8.2346e-001,
        +2.2082e-001, +0.0000e+000, +9.6177e-001, -1.5650e-002, +1.2708e-003,
        +0.0000e+000, -3.9913e-001, +2.8020e-002, +2.8334e-002, +8.5980e-004,
        +0.0000e+000, +3.0545e-001, -2.1691e-002, +6.4067e-004, -3.6528e-005,
        -1.1166e-004, +0.0000e+000, -7.6974e-002, -1.8986e-002, +5.6896e-003,
        -2.4159e-004, -2.3033e-004, -9.6783e-006, +0.0000e+000, -1.0218e-001,
        -1.3916e-002, -4.1025e-003, -5.1340e-005, -7.0114e-005, -3.3152e-007,
        +1.6901e-006, +0.0000e+000, -1.2422e-002, +2.5072e-003, +1.1205e-003,
        -1.3034e-004, -2.3971e-005, -2.6622e-006, +5.7852e-007, +4.5847e-008,
        +0.0000e+000, +4.4777e-002, -3.0421e-003, +2.6062e-005, -7.2421e-005,
        +1.9119e-006, +3.9236e-007, +2.2390e-007, +2.9765e-009, -4.6452e-009};

    double t_gpt::at_mean[55] = {
        +1.6257e+001, +2.1224e+000, +9.2569e-001, -2.5974e+001, +1.4510e+000,
        +9.2468e-002, -5.3192e-001, +2.1094e-001, -6.9210e-002, -3.4060e-002,
        -4.6569e+000, +2.6385e-001, -3.6093e-002, +1.0198e-002, -1.8783e-003,
        +7.4983e-001, +1.1741e-001, +3.9940e-002, +5.1348e-003, +5.9111e-003,
        +8.6133e-006, +6.3057e-001, +1.5203e-001, +3.9702e-002, +4.6334e-003,
        +2.4406e-004, +1.5189e-004, +1.9581e-007, +5.4414e-001, +3.5722e-001,
        +5.2763e-002, +4.1147e-003, -2.7239e-004, -5.9957e-005, +1.6394e-006,
        -7.3045e-007, -2.9394e+000, +5.5579e-002, +1.8852e-002, +3.4272e-003,
        -2.3193e-005, -2.9349e-005, +3.6397e-007, +2.0490e-006, -6.4719e-008,
        -5.2225e-001, +2.0799e-001, +1.3477e-003, +3.1613e-004, -2.2285e-004,
        -1.8137e-005, -1.5177e-007, +6.1343e-007, +7.8566e-008, +1.0749e-009};

    double t_gpt::bt_mean[55] = {
        +0.0000e+000, +0.0000e+000, +1.0210e+000, +0.0000e+000, +6.0194e-001,
        +1.2292e-001, +0.0000e+000, -4.2184e-001, +1.8230e-001, +4.2329e-002,
        +0.0000e+000, +9.3312e-002, +9.5346e-002, -1.9724e-003, +5.8776e-003,
        +0.0000e+000, -2.0940e-001, +3.4199e-002, -5.7672e-003, -2.1590e-003,
        +5.6815e-004, +0.0000e+000, +2.2858e-001, +1.2283e-002, -9.3679e-003,
        -1.4233e-003, -1.5962e-004, +4.0160e-005, +0.0000e+000, +3.6353e-002,
        -9.4263e-004, -3.6762e-003, +5.8608e-005, -2.6391e-005, +3.2095e-006,
        -1.1605e-006, +0.0000e+000, +1.6306e-001, +1.3293e-002, -1.1395e-003,
        +5.1097e-005, +3.3977e-005, +7.6449e-006, -1.7602e-007, -7.6558e-008,
        +0.0000e+000, -4.5415e-002, -1.8027e-002, +3.6561e-004, -1.1274e-004,
        +1.3047e-005, +2.0001e-006, -1.5152e-007, -2.7807e-008, +7.7491e-009};

    double t_gpt::at_amp[55] = {
        -1.8654e+000, -9.0041e+000, -1.2974e-001, -3.6053e+000, +2.0284e-002,
        +2.1872e-001, -1.3015e+000, +4.0355e-001, +2.2216e-001, -4.0605e-003,
        +1.9623e+000, +4.2887e-001, +2.1437e-001, -1.0061e-002, -1.1368e-003,
        -6.9235e-002, +5.6758e-001, +1.1917e-001, -7.0765e-003, +3.0017e-004,
        +3.0601e-004, +1.6559e+000, +2.0722e-001, +6.0013e-002, +1.7023e-004,
        -9.2424e-004, +1.1269e-005, -6.9911e-006, -2.0886e+000, -6.7879e-002,
        -8.5922e-004, -1.6087e-003, -4.5549e-005, +3.3178e-005, -6.1715e-006,
        -1.4446e-006, -3.7210e-001, +1.5775e-001, -1.7827e-003, -4.4396e-004,
        +2.2844e-004, -1.1215e-005, -2.1120e-006, -9.6421e-007, -1.4170e-008,
        +7.8720e-001, -4.4238e-002, -1.5120e-003, -9.4119e-004, +4.0645e-006,
        -4.9253e-006, -1.8656e-006, -4.0736e-007, -4.9594e-008, +1.6134e-009};

    double t_gpt::bt_amp[55] = {
        +0.0000e+000, +0.0000e+000, -8.9895e-001, +0.0000e+000, -1.0790e+000,
        -1.2699e-001, +0.0000e+000, -5.9033e-001, +3.4865e-002, -3.2614e-002,
        +0.0000e+000, -2.4310e-002, +1.5607e-002, -2.9833e-002, -5.9048e-003,
        +0.0000e+000, +2.8383e-001, +4.0509e-002, -1.8834e-002, -1.2654e-003,
        -1.3794e-004, +0.0000e+000, +1.3306e-001, +3.4960e-002, -3.6799e-003,
        -3.5626e-004, +1.4814e-004, +3.7932e-006, +0.0000e+000, +2.0801e-001,
        +6.5640e-003, -3.4893e-003, -2.7395e-004, +7.4296e-005, -7.9927e-006,
        -1.0277e-006, +0.0000e+000, +3.6515e-002, -7.4319e-003, -6.2873e-004,
        -8.2461e-005, +3.1095e-005, -5.3860e-007, -1.2055e-007, -1.1517e-007,
        +0.0000e+000, +3.1404e-002, +1.5580e-002, -1.1428e-003, +3.3529e-005,
        +1.0387e-005, -1.9378e-006, -2.7327e-007, +7.5833e-009, -9.2323e-009};

    // GPT empirical model v1
    int t_gpt::gpt_v1(double dmjd, double dlat, double dlon, double dhgt,
                      double &pres, double &temp, double &undu)
    {

        pres = temp = undu = 0.0;
        int i = 0;
        int m = 0;
        int n = 0;

        // reference day is 28 January
        // this is taken from Niell (1996) to be consistent
        // for constant values use: doy = 91.3125
        double doy = dmjd - 44239.0 + 1 - 28;

        //degree n and order m (=n)
        int nmax = 9;

        // unit vector
        double x = cos(dlat) * cos(dlon);
        double y = cos(dlat) * sin(dlon);
        double z = sin(dlat);

        double V[10][10];
        double W[10][10];

        // Legendre polynomials
        V[0][0] = 1.0;
        W[0][0] = 0.0;
        V[1][0] = z * V[0][0];
        W[1][0] = 0.0;

        for (n = 2; n <= nmax; n++)
        {
            V[n][0] = ((2 * n - 1) * z * V[n - 1][0] - (n - 1) * V[n - 2][0]) / n;
            W[n][0] = 0.0;
        }

        for (m = 1; m <= nmax; m++)
        {
            V[m][m] = (2 * m - 1) * (x * V[m - 1][m - 1] - y * W[m - 1][m - 1]);
            W[m][m] = (2 * m - 1) * (x * W[m - 1][m - 1] + y * V[m - 1][m - 1]);

            if (m < nmax)
            {
                V[m + 1][m] = (2 * m + 1) * z * V[m][m];
                W[m + 1][m] = (2 * m + 1) * z * W[m][m];
            }

            for (n = m + 2; n <= nmax; n++)
            {
                V[n][m] = ((2 * n - 1) * z * V[n - 1][m] - (n + m - 1) * V[n - 2][m]) / (n - m);
                W[n][m] = ((2 * n - 1) * z * W[n - 1][m] - (n + m - 1) * W[n - 2][m]) / (n - m);
            }
        }

        // Geoidal height
        i = 0;
        for (n = 0; n <= nmax; n++)
        {
            for (m = 0; m <= n; m++)
            {
                undu += a_geoid[i] * V[n][m] + b_geoid[i] * W[n][m];
                i++;
            }
        }

        // Orthometric height
        double hort = dhgt - undu;

        // Surface temperature on the geoid
        double atm = 0.0;
        double ata = 0.0;
        i = 0;
        for (n = 0; n <= nmax; n++)
        {
            for (m = 0; m <= n; m++)
            {
                atm += at_mean[i] * V[n][m] + bt_mean[i] * W[n][m];
                ata += at_amp[i] * V[n][m] + bt_amp[i] * W[n][m];
                i++;
            }
        }
        double temp0 = atm + ata * cos(doy / 365.25 * 2.0 * G_PI);

        // Height correction for temperature
        temp = temp0 - 0.0065 * hort;

        // Surface pressure on the geoid
        double apm = 0.0;
        double apa = 0.0;
        i = 0;
        for (n = 0; n <= nmax; n++)
        {
            for (m = 0; m <= n; m++)
            {
                apm += ap_mean[i] * V[n][m] + bp_mean[i] * W[n][m];
                apa += ap_amp[i] * V[n][m] + bp_amp[i] * W[n][m];
                i++;
            }
        }
        double pres0 = apm + apa * cos(doy / 365.25 * 2.0 * G_PI);

        // Height correction for pressure (Berg)
        pres = pres0 * pow(1.0 - 0.0000226 * hort, 5.225);

        // Height correction for pressure (P+T dependant)
        //   pres = pres0*exp(-9.80665*hort/287.0/(temp+273.15));

        return 0;
    }

    bool t_gpt3::getGPT3Data(string GPT3path, double mjd, double lat, double lon, double h_ell, int it
        , double* p, double* T, double* dT, double* Tm, double* e, double* ah, double* aw
        , double* la, double* undu, double* Gn_h, double* Ge_h, double* Gn_w, double* Ge_w)
	{
		FILE* fp = fopen(GPT3path.c_str(), "r");
		if (!fp)
		{
			//printf("\n------- Open GPT3 File failed!! -------\n\n");
			fclose(fp); fp == NULL;
			return false;
		}

		if (fabs(lat) > G_PI / 2)
		{
			//printf("\n------ Func. getGPT3Data, lat is out of  range!! -------\n\n");
			fclose(fp); fp == NULL;
			return false;
		}
		if (fabs(lon) > 2 * G_PI || fabs(lon) < -G_PI / 2)
		{
			//printf("\n------ Func. getGPT3Data, lon is out of  range!! -------\n\n");
			fclose(fp); fp == NULL;
			return false;
		}
		if (h_ell < -1000.0 || h_ell > 10000.0)
		{
			//printf("\n------ Func. getGPT3Data, h_ell is out of  range!! -------\n\n");
			fclose(fp); fp == NULL;
			return false;
		}

		int iyear, idoy;
		char line[512]; //GPT3һ����442��
		int nword = 0;
		char* word[70]; //GPT3һ����60������
		double data[70];
		int lineLen;//GPT3һ�еĳ���

		double p_grid[4][5] = { 0 };
		double T_grid[4][5] = { 0 };
		double Q_grid[4][5] = { 0 };
		double dT_grid[4][5] = { 0 };
		double u_grid[4][1] = { 0 };
		double Hs_grid[4][1] = { 0 };
		double ah_grid[4][5] = { 0 };
		double aw_grid[4][5] = { 0 };
		double la_grid[4][5] = { 0 };
		double Tm_grid[4][5] = { 0 };
		double Gn_h_grid[4][5] = { 0 };
		double Ge_h_grid[4][5] = { 0 };
		double Gn_w_grid[4][5] = { 0 };
		double Ge_w_grid[4][5] = { 0 };

		mjd2doy((int)(mjd), &iyear, &idoy);
		double doy = mjd - (int)(mjd)+idoy;
		//determine the GPT3 coefficients

		//mean gravity in m / s * *2
		double gm = 9.80665;
		// molar mass of dry air in kg / mol
		double dMtr = 28.965 * 0.001;
		// universal gas constant in J / K / mol
		double Rg = 8.3143;

		// factors for amplitudes
		double cosfy = 0.0;
		double coshy = 0.0;
		double sinfy = 0.0;
		double sinhy = 0.0;
		if (it == 1) // then  constant parameters
		{
			cosfy = 0;
			coshy = 0;
			sinfy = 0;
			sinhy = 0;
		}
		else
		{
			cosfy = cos(doy / 365.25 * 2 * G_PI);// coefficient for A1
			coshy = cos(doy / 365.25 * 4 * G_PI);// coefficient for B1
			sinfy = sin(doy / 365.25 * 2 * G_PI);// coefficient for A2
			sinhy = sin(doy / 365.25 * 4 * G_PI);// coefficient for B2
		}

		// only positive longitude in degrees
		double plon;
		if (lon < 0)
		{
			plon = (lon + 2 * G_PI) * 180 / G_PI;
		}
		else
		{
			plon = lon * 180 / G_PI;
		}

		// transform to polar distance in degrees
		double ppod = (-lat + G_PI / 2) * 180 / G_PI;
		// find the index(line in the grid file) of the nearest point
		//changed for the 1 degree grid
		int ipod = floor(ppod + 1);
		int ilon = floor(plon + 1);

		// normalized(to one) differences, can be positive or negative
		// changed for the 1 degree grid
		double diffpod = (ppod - (ipod - 0.5));
		double difflon = (plon - (ilon - 0.5));
		// changed for the 1 degree grid
		if (ipod == 181) ipod = 180;
		if (ilon == 361) ilon = 1;
		if (ilon == 0) ilon = 360;
		//get the number of the corresponding line
		// changed for the 1 degree grid
		int indx[4] = { 0 };
		indx[0] = (ipod - 1) * 360 + ilon;

		// near the poles : nearest neighbour interpolation, otherwise : bilinear
		// with the 1 degree grid the limits are lower and upper
		int bilinear = 0;
		if (ppod > 0.5 && ppod < 179.5) bilinear = 1;
		if (bilinear == 0)
		{
			int ix = indx[0];
			rewind(fp);
			fgets(line, 442, fp);
			fseek(fp, 442 * ix, SEEK_SET);
			fgets(line, 442, fp);
			lineLen = strlen(line);
			split_string(true, line, line + lineLen, " ", &nword, word);
			for (int j = 0; j < nword; j++)
			{
				data[j] = atof(word[j]);
				free(word[j]);
			}
			gCopyArray(&p_grid[0][0], &data[2], 5);
			gCopyArray(&T_grid[0][0], &data[7], 5);
			gCopyArray(&Q_grid[0][0], &data[12], 5);
			gCopyArray(&dT_grid[0][0], &data[17], 5);
			gCopyArray(&u_grid[0][0], &data[22], 1);
			gCopyArray(&Hs_grid[0][0], &data[23], 1);
			gCopyArray(&ah_grid[0][0], &data[24], 5);
			gCopyArray(&aw_grid[0][0], &data[29], 5);
			gCopyArray(&la_grid[0][0], &data[34], 5);
			gCopyArray(&Tm_grid[0][0], &data[39], 5);
			gCopyArray(&Gn_h_grid[0][0], &data[44], 5);
			gCopyArray(&Ge_h_grid[0][0], &data[49], 5);
			gCopyArray(&Gn_w_grid[0][0], &data[54], 5);
			gCopyArray(&Ge_w_grid[0][0], &data[59], 5);
			;
			for (int j = 0; j < 4; j++)
			{
				for (int k = 0; k < 5; k++)
				{
					Q_grid[j][k] /= 1000;
					dT_grid[j][k] /= 1000;
					ah_grid[j][k] /= 1000;
					aw_grid[j][k] /= 1000;
					Gn_h_grid[j][k] /= 100000;
					Ge_h_grid[j][k] /= 100000;
					Gn_w_grid[j][k] /= 100000;
					Ge_w_grid[j][k] /= 100000;
				}
			}

			// transforming ellipsoidal height to orthometric height
			double _undu = u_grid[ix][0];
			double hgt = h_ell - _undu;

			// pressure, temperature at the height of the grid
			double T0 = T_grid[ix][0] + T_grid[ix][1] * cosfy + T_grid[ix][2] * sinfy + T_grid[ix][3] * coshy + T_grid[ix][4] * sinhy;
			double p0 = p_grid[ix][0] + p_grid[ix][1] * cosfy + p_grid[ix][2] * sinfy + p_grid[ix][3] * coshy + p_grid[ix][4] * sinhy;
			// specific humidity
			double Q = Q_grid[ix][0] + Q_grid[ix][1] * cosfy + Q_grid[ix][2] * sinfy + Q_grid[ix][3] * coshy + Q_grid[ix][4] * sinhy;

			// lapse rate of the temperature
			double _dT = dT_grid[ix][0] + dT_grid[ix][1] * cosfy + dT_grid[ix][2] * sinfy + dT_grid[ix][3] * coshy + dT_grid[ix][4] * sinhy;

			// station height - grid height
			double redh = hgt - Hs_grid[ix][0];

			// temperature at station height in Celsius
			double _T = T0 + _dT * redh - 273.15;

			// temperature lapse rate in degrees / km
			_dT = _dT * 1000;

			// virtual temperature in Kelvin
			double Tv = T0 * (1 + 0.6077 * Q);

			double c = gm * dMtr / (Rg * Tv);
			// pressure in hPa
			double _p = (p0 * exp(-c * redh)) / 100;

			// hydrostaticand wet coefficients ahand aw
			double _ah = ah_grid[ix][0] + ah_grid[ix][1] * cosfy + ah_grid[ix][2] * sinfy
				+ ah_grid[ix][3] * coshy + ah_grid[ix][4] * sinhy;
			double _aw = aw_grid[ix][0] + aw_grid[ix][1] * cosfy + aw_grid[ix][2] * sinfy
				+ aw_grid[ix][3] * coshy + aw_grid[ix][4] * sinhy;

			// water vapour decrease factor la
			double _la = la_grid[ix][0] + la_grid[ix][1] * cosfy
				+ la_grid[ix][2] * sinfy + la_grid[ix][3] * coshy
				+ la_grid[ix][4] * sinhy;

			// mean temperature Tm
			double _Tm = Tm_grid[ix][0] + Tm_grid[ix][1] * cosfy
				+ Tm_grid[ix][2] * sinfy + Tm_grid[ix][3] * coshy
				+ Tm_grid[ix][4] * sinhy;

			// northand east gradients(total, hydrostaticand wet)
			double _Gn_h = Gn_h_grid[ix][0] + Gn_h_grid[ix][1] * cosfy + Gn_h_grid[ix][2] * sinfy + Gn_h_grid[ix][3] * coshy + Gn_h_grid[ix][4] * sinhy;
			double _Ge_h = Ge_h_grid[ix][0] + Ge_h_grid[ix][1] * cosfy + Ge_h_grid[ix][2] * sinfy + Ge_h_grid[ix][3] * coshy + Ge_h_grid[ix][4] * sinhy;
			double _Gn_w = Gn_w_grid[ix][0] + Gn_w_grid[ix][1] * cosfy + Gn_w_grid[ix][2] * sinfy + Gn_w_grid[ix][3] * coshy + Gn_w_grid[ix][4] * sinhy;
			double _Ge_w = Ge_w_grid[ix][0] + Ge_w_grid[ix][1] * cosfy + Ge_w_grid[ix][2] * sinfy + Ge_w_grid[ix][3] * coshy + Ge_w_grid[ix][4] * sinhy;

			// water vapor pressure in hPa
			double e0 = Q * p0 / (0.622 + 0.378 * Q) / 100;// on the grid
			double _e = e0 * pow(100 * _p / p0, _la + 1);// on the station height - (14) Askne and Nordius, 1987

			//��ֵ
			// pressure
			if (p) *p = _p;

			// temperature
			if (T) *T = _T;

			// temperature in degree per km
			if (dT) *dT = _dT;

			// water vapor pressure in hPa
			if (e) *e = _e;

			// ahand aw
			if (ah) *ah = _ah;
			if (aw) *aw = _aw;

			// undulation
			if (undu) *undu = _undu;

			// water vapor decrease factor la
			if (la) *la = _la;

			// gradients
			if (Gn_h) *Gn_h = _Gn_h;
			if (Ge_h) *Ge_h = _Ge_h;
			if (Gn_w) *Gn_w = _Gn_w;
			if (Ge_w) *Ge_w = _Ge_w;

			// mean temperature of the water vapor Tm
			if (Tm) *Tm = _Tm;
		}
		else
		{
			int ipod1 = ipod + (diffpod < 0 ? -1 : 1);
			int ilon1 = ilon + (difflon < 0 ? -1 : 1);
			if (ilon1 == 361) ilon1 = 1;
			if (ilon1 == 0) ilon1 = 360;
			// get the number of the line
				// changed for the 1 degree grid
			indx[1] = (ipod1 - 1) * 360 + ilon;// along same longitude
			indx[2] = (ipod - 1) * 360 + ilon1;// along same polar distance
			indx[3] = (ipod1 - 1) * 360 + ilon1;// diagonal

			//ѭ����ȡ���������
			for (int i = 0; i < 4; i++)
			{
				rewind(fp);
				fgets(line, 442, fp);
				fseek(fp, 442 * indx[i], SEEK_SET);
				fgets(line, 442, fp);
				lineLen = strlen(line);
				split_string(true, line, line + lineLen, " ", &nword, word);
				for (int j = 0; j < nword; j++)
				{
					data[j] = atof(word[j]);
					free(word[j]);
				}
				gCopyArray(&p_grid[i][0], &data[2], 5);
				gCopyArray(&T_grid[i][0], &data[7], 5);
				gCopyArray(&Q_grid[i][0], &data[12], 5);
				gCopyArray(&dT_grid[i][0], &data[17], 5);
				gCopyArray(&u_grid[i][0], &data[22], 1);
				gCopyArray(&Hs_grid[i][0], &data[23], 1);
				gCopyArray(&ah_grid[i][0], &data[24], 5);
				gCopyArray(&aw_grid[i][0], &data[29], 5);
				gCopyArray(&la_grid[i][0], &data[34], 5);
				gCopyArray(&Tm_grid[i][0], &data[39], 5);
				gCopyArray(&Gn_h_grid[i][0], &data[44], 5);
				gCopyArray(&Ge_h_grid[i][0], &data[49], 5);
				gCopyArray(&Gn_w_grid[i][0], &data[54], 5);
				gCopyArray(&Ge_w_grid[i][0], &data[59], 5);
			}
			for (int j = 0; j < 4; j++)
			{
				for (int k = 0; k < 5; k++)
				{
					Q_grid[j][k] /= 1000;
					dT_grid[j][k] /= 1000;
					ah_grid[j][k] /= 1000;
					aw_grid[j][k] /= 1000;
					Gn_h_grid[j][k] /= 100000;
					Ge_h_grid[j][k] /= 100000;
					Gn_w_grid[j][k] /= 100000;
					Ge_w_grid[j][k] /= 100000;
				}
			}
			//transforming ellipsoidal height to orthometric height : Hortho = -N + Hell
			double undul[4] = { u_grid[0][0],u_grid[0][1],u_grid[0][2],u_grid[0][3] };
			double hgt[4] = { h_ell - undul[0], h_ell - undul[1], h_ell - undul[2], h_ell - undul[3] };

			//at the height of the grid
			double T0[4] = { 0.0 }; //temperature
			double p0[4] = { 0.0 };// pressure
			double Ql[4] = { 0.0 };// humidity
			double Hs1[4] = { 0.0 };// 
			double redh[4] = { 0.0 };//% reduction = stationheight - gridheight
			double dTl[4] = { 0.0 };//lapse rate of the temperature in degree / m
			double Tl[4] = { 0.0 }; // temperature reduction to station height
			double Tv[4] = { 0.0 }; // virtual temperature
			double c[4] = { 0.0 };
			double pl[4] = { 0.0 }; // pressure in hPa
			double ahl[4] = { 0.0 };// hydrostaticand wet coefficients ah and aw
			double awl[4] = { 0.0 };
			double lal[4] = { 0.0 }; // water vapour decrease factor la
			double Tml[4] = { 0.0 }; // mean temperature of the water vapor Tm
			double Gn_hl[4] = { 0.0 };// northand east gradients(total, hydrostaticand wet)
			double Ge_hl[4] = { 0.0 };
			double Gn_wl[4] = { 0.0 };
			double Ge_wl[4] = { 0.0 };
			double e0[4] = { 0.0 }; // water vapor pressure in hPa
			double el[4] = { 0.0 };
			for (int j = 0; j < 4; j++)
			{
				// pressure, temperature at the height of the grid
				T0[j] = T_grid[j][0] + T_grid[j][1] * cosfy
					+ T_grid[j][2] * sinfy + T_grid[j][3] * coshy + T_grid[j][4] * sinhy;
				p0[j] = p_grid[j][0] + p_grid[j][1] * cosfy
					+ p_grid[j][2] * sinfy + p_grid[j][3] * coshy + p_grid[j][4] * sinhy;
				// humidity
				Ql[j] = Q_grid[j][0] + Q_grid[j][1] * cosfy
					+ Q_grid[j][2] * sinfy + Q_grid[j][3] * coshy + Q_grid[j][4] * sinhy;
				//reduction = stationheight - gridheight
				Hs1[j] = Hs_grid[j][0];
				gArrayMinus(redh, hgt, Hs1, 4);

				//lapse rate of the temperature in degree / m
				dTl[j] = dT_grid[j][0] + dT_grid[j][1] * cosfy
					+ dT_grid[j][2] * sinfy + dT_grid[j][3] * coshy + dT_grid[j][4] * sinhy;

				// temperature reduction to station height
				// height reduction
				Tl[j] = T0[j] + dTl[j] * redh[j] - 273.15;
				// virtual temperature
				Tv[j] = T0[j] * (1 + 0.6077 * Ql[j]);
				c[j] = gm * dMtr / Rg / Tv[j];

				// pressure in hPa
				// height reduction
				pl[j] = p0[j] * exp(-c[j] * redh[j]) / 100;


				// hydrostaticand wet coefficients ahand aw
				ahl[j] = ah_grid[j][0] + ah_grid[j][1] * cosfy
					+ ah_grid[j][2] * sinfy + ah_grid[j][3] * coshy + ah_grid[j][4] * sinhy;
				awl[j] = aw_grid[j][0] + aw_grid[j][1] * cosfy
					+ aw_grid[j][2] * sinfy + aw_grid[j][3] * coshy + aw_grid[j][4] * sinhy;

				// water vapour decrease factor la
				lal[j] = la_grid[j][0] + la_grid[j][1] * cosfy
					+ la_grid[j][2] * sinfy + la_grid[j][3] * coshy + la_grid[j][4] * sinhy;

				// mean temperature of the water vapor Tm
				Tml[j] = Tm_grid[j][0] + Tm_grid[j][1] * cosfy
					+ Tm_grid[j][2] * sinfy + Tm_grid[j][3] * coshy + Tm_grid[j][4] * sinhy;

				// northand east gradients(total, hydrostatic and wet)
				Gn_hl[j] = Gn_h_grid[j][0] + Gn_h_grid[j][1] * cosfy
					+ Gn_h_grid[j][2] * sinfy + Gn_h_grid[j][3] * coshy + Gn_h_grid[j][4] * sinhy;
				Ge_hl[j] = Ge_h_grid[j][0] + Ge_h_grid[j][1] * cosfy
					+ Ge_h_grid[j][2] * sinfy + Ge_h_grid[j][3] * coshy + Ge_h_grid[j][4] * sinhy;
				Gn_wl[j] = Gn_w_grid[j][0] + Gn_w_grid[j][1] * cosfy
					+ Gn_w_grid[j][2] * sinfy + Gn_w_grid[j][3] * coshy + Gn_w_grid[j][4] * sinhy;
				Ge_wl[j] = Ge_w_grid[j][0] + Ge_w_grid[j][1] * cosfy
					+ Ge_w_grid[j][2] * sinfy + Ge_w_grid[j][3] * coshy + Ge_w_grid[j][4] * sinhy;

				//water vapor pressure in hPa
				// height reduction
				e0[j] = Ql[j] * p0[j] / (0.622 + 0.378 * Ql[j]) / 100;//on the grid
				el[j] = e0[j] * pow((100 * pl[j] / p0[j]), (lal[j] + 1));// on the station height - (14) Askne and Nordius, 1987
			}

			double dnpod1 = fabs(diffpod);// distance nearer point
			double dnpod2 = 1 - dnpod1;// distance to distant point
			double dnlon1 = fabs(difflon);
			double dnlon2 = 1 - dnlon1;

			// pressure
			double R1 = dnpod2 * pl[0] + dnpod1 * pl[1];
			double R2 = dnpod2 * pl[2] + dnpod1 * pl[3];
			if (p)
			{
				*p = dnlon2 * R1 + dnlon1 * R2;
			}

			// temperature
			R1 = dnpod2 * Tl[0] + dnpod1 * Tl[1];
			R2 = dnpod2 * Tl[2] + dnpod1 * Tl[3];
			if (T) *T = dnlon2 * R1 + dnlon1 * R2;

			// temperature in degree per km
			R1 = dnpod2 * dTl[0] + dnpod1 * dTl[1];
			R2 = dnpod2 * dTl[2] + dnpod1 * dTl[3];
			if (dT) *dT = (dnlon2 * R1 + dnlon1 * R2) * 1000;

			// water vapor pressure in hPa
			R1 = dnpod2 * el[0] + dnpod1 * el[1];
			R2 = dnpod2 * el[2] + dnpod1 * el[3];
			if (e) *e = dnlon2 * R1 + dnlon1 * R2;

			// ah and aw
			R1 = dnpod2 * ahl[0] + dnpod1 * ahl[1];
			R2 = dnpod2 * ahl[2] + dnpod1 * ahl[3];
			if (ah) *ah = dnlon2 * R1 + dnlon1 * R2;
			R1 = dnpod2 * awl[0] + dnpod1 * awl[1];
			R2 = dnpod2 * awl[2] + dnpod1 * awl[3];
			if (aw) *aw = dnlon2 * R1 + dnlon1 * R2;

			// undulation
			R1 = dnpod2 * undul[0] + dnpod1 * undul[1];
			R2 = dnpod2 * undul[2] + dnpod1 * undul[3];
			if (undu) *undu = dnlon2 * R1 + dnlon1 * R2;

			// water vapor decrease factor la
			R1 = dnpod2 * lal[0] + dnpod1 * lal[1];
			R2 = dnpod2 * lal[2] + dnpod1 * lal[3];
			if (la) *la = dnlon2 * R1 + dnlon1 * R2;

			// gradients
			R1 = dnpod2 * Gn_hl[0] + dnpod1 * Gn_hl[1];
			R2 = dnpod2 * Gn_hl[2] + dnpod1 * Gn_hl[3];
			if (Gn_h) *Gn_h = (dnlon2 * R1 + dnlon1 * R2);
			R1 = dnpod2 * Ge_hl[0] + dnpod1 * Ge_hl[1];
			R2 = dnpod2 * Ge_hl[2] + dnpod1 * Ge_hl[3];
			if (Ge_h) *Ge_h = (dnlon2 * R1 + dnlon1 * R2);
			R1 = dnpod2 * Gn_wl[0] + dnpod1 * Gn_wl[1];
			R2 = dnpod2 * Gn_wl[2] + dnpod1 * Gn_wl[3];
			if (Gn_w) *Gn_w = (dnlon2 * R1 + dnlon1 * R2);
			R1 = dnpod2 * Ge_wl[0] + dnpod1 * Ge_wl[1];
			R2 = dnpod2 * Ge_wl[2] + dnpod1 * Ge_wl[3];
			if (Ge_w) *Ge_w = (dnlon2 * R1 + dnlon1 * R2);

			// mean temperature of the water vapor Tm
			R1 = dnpod2 * Tml[0] + dnpod1 * Tml[1];
			R2 = dnpod2 * Tml[2] + dnpod1 * Tml[3];
			if (Tm) *Tm = dnlon2 * R1 + dnlon1 * R2;
		}
		fclose(fp); fp == NULL;

		return 1;
	}

	void t_gpt3::gCopyArray(double* destination, const double* source, int n)
	{
		for (int i = 0; i < n; i++)
		{
			destination[i] = source[i];
		}
	}

	void t_gpt3::gArrayMinus(double* destination, const double* source, double* minus, int n)
	{
		int i;
		for (i = 0; i < n; i++)
		{
			destination[i] = source[i] - minus[i];
		}
	}

	void t_gpt3::split_string(bool lnoempty, char* start, char* end, const char* separator, int* nword, char** word)
	{

		int count = 0;
		char msg[512];
		int i = 0;
		int j = 0;
		int len = 0;
		char* strtmp[512] = { 0 };

		char* src;
		src = (char*)calloc(512, sizeof(char));

		if (start == NULL || strlen(start) == 0)
		{
			free(src);
			return;
		}

		if (end - start > 512)
		{
			sprintf(msg, "***ERROR(split_line): input string length > 512");
			//CPSstop(msg);
		}
		memcpy(src, start, end - start);
		len = strlen(src);

		for (i = 0; i < len; i++)
		{
			for (j = 0; j < strlen(separator); j++)
			{
				if (separator[j] == src[i])
				{
					src[i] = '\0';
					break;
				}
			}
		}

		*strtmp = src;
		count++;
		for (i = 0; i < len; i++)
		{
			if ('\0' == src[i])
			{
				*(strtmp + count) = src + i + 1;
				count++;
			}
		}

		*nword = 0;
		for (i = 0; i < count; i++)
		{
			if (true == lnoempty)
			{
				if (0 != strcmp("", strtmp[i]))
				{
					word[*nword] = (char*)calloc(512, sizeof(char)); // to debug
					strcpy(word[*nword], strtmp[i]);
					(*nword)++;
				}
			}
			else
			{
				if (0 == strcmp("", strtmp[i]))
				{
					word[*nword] = (char*)calloc(512, sizeof(char));// to debug
					strcpy(word[*nword], " ");
					(*nword)++;
				}
				else
				{
					word[*nword] = (char*)calloc(512, sizeof(char));// to debug
					strcpy(word[*nword], strtmp[i]);
					(*nword)++;
				}
			}
		}

		free(src);
	}

	int t_gpt3::modified_julday(int iday, int imonth, int iyear)
	{
		static int doy_of_month[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
		int iyr = 0, modified_julday = 0;
		//check the input data
		if ((iyear < 0 || imonth < 0 || iday < 0 || imonth>12 || iday>366) || (0 != imonth && iday > 31))
		{

		}
		iyr = iyear;
		if (imonth <= 2)
		{
			iyr = iyr - 1;
		}
		modified_julday = 365 * iyear - 678941 + iyr / 4 - iyr / 100 + iyr / 400 + iday;
		if (0 != imonth)
		{
			modified_julday = modified_julday + doy_of_month[imonth - 1];
		}
		return modified_julday;
	}

	void t_gpt3::mjd2doy(int jd, int* iyear, int* idoy)
	{
		*iyear = (jd + 678940) / 365;
		*idoy = jd - modified_julday(1, 1, *iyear);
		while (*idoy <= 0)
		{
			*iyear = *iyear - 1;
			*idoy = jd - modified_julday(1, 1, *iyear) + 1;
		}
	}

	void t_gpt3::GPT3_map(double ah, double aw, double mjd, double lat, double lon, double h_ell, double zd, double* dmap, double* wmap)
	{
		const static double anm_bh_A0[91] =
		{ 0.00271286, 9.80477e-06,-1.03443e-05,-3.00354e-05,-7.99121e-07,
	  7.5144e-07, 2.21791e-05, -4.4764e-06, 8.14993e-07,-7.96928e-08,
	-2.16138e-05,  5.3436e-07, 6.80236e-07,-7.13791e-08,-5.80109e-09,
	 1.75559e-05,-1.90789e-06,  2.2705e-07,-2.77029e-08,-1.89918e-08,
	 1.04022e-09,-3.11579e-05, 6.00037e-07,-9.79584e-08,-2.37742e-08,
	-6.58203e-09, 9.45858e-10, 1.61377e-10, 2.99634e-07,-6.52698e-07,
	-1.62727e-07, -2.9637e-09,-9.91102e-10, 1.21268e-10, 6.92013e-11,
	 5.56731e-12, 3.96883e-06, 1.10123e-06,-3.40345e-08,-6.49732e-09,
	 5.45176e-10, 1.89481e-10, 2.10126e-12,-8.07895e-12, 1.38737e-12,
	 6.38137e-06,-2.25321e-07, 3.48895e-09,-9.66683e-11, 1.06834e-09,
	 5.22434e-11, 2.13396e-13,-1.24352e-12,-5.39024e-13, 3.12768e-14,
	 2.08311e-06,-3.94602e-07, 6.11068e-08, 2.57771e-09, 7.16926e-10,
	 4.90008e-12, 9.75489e-13,-3.79534e-13,-3.96463e-14,-8.64942e-15,
	-4.13816e-15,-8.34852e-07,-2.75514e-07,-4.26696e-08, 1.12408e-09,
	 1.76612e-10,-9.35961e-12,-3.34624e-12,-6.16776e-14,-8.50729e-15,
	 4.39043e-15,-5.36055e-16,-3.26422e-17, 4.67238e-06,-3.33739e-07,
	 6.64201e-09,  5.2555e-09, 1.89318e-10,-1.02253e-12, -2.6977e-12,
	 1.07832e-13, 6.29825e-15,-5.57346e-16,  1.7322e-16,-7.30829e-18,
	  5.6204e-18 };
		const static double anm_bh_A1[91] =
		{
		-1.39198e-06,-5.83923e-05,-2.05536e-06, 2.37284e-05,-5.39048e-06,
	  3.8551e-07,-5.56986e-05,-2.60453e-06, 2.03946e-07,-3.66953e-08,
	  1.6735e-05,-3.64189e-06, 1.33023e-07, 7.81999e-09, 1.13702e-09,
	-2.85579e-05,-1.62145e-06, 1.03926e-07,-5.96251e-09, 3.54083e-09,
	-2.36437e-10,-3.43576e-06,-1.12539e-07, 9.14964e-08,-5.53336e-09,
	 9.28456e-10,-3.29683e-11, 6.84326e-12, 8.14392e-06, 2.87255e-07,
	 1.09102e-07, 1.20008e-09, 7.54542e-10,-5.93573e-11,-9.02885e-12,
	 3.57272e-12,-4.57101e-06, 4.58047e-07,  2.8765e-08,-3.07048e-09,
	-3.73288e-10, 3.67431e-12,-3.08421e-12, 9.19345e-13,  1.0934e-13,
	 4.62426e-06, 3.21908e-07, 3.49951e-08, 3.75807e-09,-4.07975e-10,
	 6.02012e-12,-1.71939e-12, 8.26106e-13, 2.54188e-14,-2.29518e-15,
	-1.22179e-06,-5.44015e-07, -3.9324e-08,   1.878e-09,-3.65858e-10,
	 3.31649e-12,-1.68968e-13, 2.76682e-13, 2.94156e-14,-1.91823e-15,
	-7.43602e-17,-7.45795e-06, 3.64714e-08,  -4.656e-09, 7.62143e-10,
	-1.01189e-10, 1.90894e-12, 4.56385e-14, 8.25317e-14, 8.38162e-15,
	 -3.6944e-16, 3.29794e-17, 7.38235e-18, 1.93611e-06,  4.1214e-07,
	 1.45412e-08,-1.33677e-09, 9.23092e-11,-7.25921e-13, -2.1785e-14,
	 2.26682e-14, 8.04458e-16,-2.93949e-16, 2.05017e-17, 2.03711e-18,
	-1.38144e-19 };
		const static double anm_bh_B1[91] =
		{
		 1.34956e-06,-2.07307e-05, 2.09693e-06, 2.02237e-05,-4.21235e-06,
	 4.41508e-08,-1.81288e-05, 2.56376e-06, 1.11832e-08,-6.74743e-09,
	 1.93768e-05,-2.99935e-06, -1.8035e-08, 1.13827e-09, 7.29046e-10,
	-1.47442e-05, 7.57239e-07,-3.31105e-09, 2.95987e-10, 8.10618e-10,
	-2.25111e-10, 5.81664e-06,-3.86745e-07,  4.7778e-09,-3.73625e-09,
	 2.47219e-10, -8.1544e-11,-4.66899e-12, 6.70458e-06,-1.78228e-08,
	 4.97827e-09, 1.75885e-09,-2.95881e-10,-5.03295e-11,-3.44152e-12,
	 2.25886e-12,-3.30208e-06, 1.86832e-07, -1.8393e-08,-2.84381e-09,
	-1.16293e-10, -1.7218e-11,-4.87749e-12, 3.26929e-13, 5.15714e-14,
	 4.42334e-06,-3.34835e-07,-6.01129e-09, 2.38984e-09,-2.37599e-10,
	-7.80605e-12,-3.57625e-14, 4.63607e-13, 1.30934e-14, 2.53117e-16,
	-2.98842e-06,-6.16955e-08,-1.62979e-08, 1.95408e-09,-1.54864e-11,
	 4.81665e-13, 7.38383e-13,  1.2148e-13, 6.78413e-15,-8.01725e-16,
	-5.54249e-17,-6.58133e-06,-7.12385e-08, 7.35038e-09,-6.72564e-10,
	 7.32546e-11,-6.34793e-13, 2.72561e-13, 7.19846e-14, 3.43652e-15,
	 1.22249e-16, 2.13564e-17, 2.49292e-18, 9.39035e-07, 1.58755e-07,
	 1.82499e-08, 7.60804e-11, 5.51435e-11, -1.9811e-12, 4.73041e-13,
	 2.56046e-14, 9.53174e-16, 7.47621e-18, 9.08874e-18, 7.62163e-19,
	 1.68029e-20 };
		const static double anm_bh_A2[91] =
		{ 2.71686e-07, 1.14629e-06,-1.55491e-08, 1.69276e-06,-2.70944e-06,
	-2.07508e-08,-4.41076e-06, 4.41601e-07, 3.25757e-08,-1.30316e-08,
	 1.99595e-06,-2.06881e-06, 2.51276e-08, -5.8963e-09,-9.10469e-10,
	  -6.293e-06, 6.93366e-07, 2.88066e-08,-5.87644e-09, 4.99207e-10,
	 -7.3985e-11, 8.31535e-07,-3.88219e-07, 2.44284e-09,-1.92304e-09,
	 1.10665e-10,-1.21616e-12, 2.31211e-12,-9.92543e-07, 2.65525e-07,
	  7.8665e-11,-1.74757e-09, 1.81009e-10, 3.05383e-11, 2.03165e-12,
	-2.44508e-13,   3.326e-06,-1.60093e-07,-9.74179e-09, 1.55123e-09,
	 7.25846e-11, 1.47046e-11, 1.16364e-14, 2.00438e-13,-5.92156e-14,
	 1.15375e-06,-4.82133e-07, 4.78214e-09, 2.07545e-09, 5.89327e-11,
	 1.50873e-11,-5.01675e-14, 6.39518e-14, 6.06153e-15, 7.07505e-16,
	  3.0731e-06,-2.31955e-07, 1.01339e-08, 1.15276e-09,  6.5077e-11,
	 7.26081e-12,-3.58436e-13,-1.57729e-14,-4.12136e-15, 5.02941e-16,
	-4.83999e-17,-1.38608e-06,-7.86206e-08, 1.17098e-08,-1.18095e-10,
	 1.79543e-11, 3.98598e-12,-3.57943e-15,-2.92416e-14,-8.19429e-16,
	-2.09359e-16,-1.37839e-18, 8.18253e-19,-5.84565e-07, 1.37449e-07,
	 2.86634e-09,-1.07919e-10, 3.86696e-11,-2.18965e-13, 1.57947e-13,
	-1.14996e-14, 1.16892e-15,-5.36417e-17,-2.86882e-18,-7.54848e-19,
	 1.81224e-19 };
		const static double anm_bh_B2[91] =
		{ 1.56659e-06,  4.9361e-06,-1.89706e-07, 8.72157e-07,-6.80894e-07,
	 4.95355e-08, 4.93573e-06, 2.93438e-07, 3.01029e-08,-2.00749e-09,
	-2.42464e-06,-9.40816e-07,-1.43241e-09,-4.20761e-09,-2.58814e-10,
	-5.12205e-07, 6.88856e-07,-8.00257e-09,-3.28804e-09,-1.52692e-10,
	 7.95929e-11,  4.0262e-06,-6.83765e-07,-6.26361e-09,-7.18681e-09,
	-4.20391e-11,-9.70713e-12, 2.39195e-12,-3.04078e-06, 8.60651e-08,
	-6.67194e-09, 3.21963e-09, 8.31547e-11,  3.5628e-11,-5.44852e-12,
	-6.83314e-13, 4.26539e-06,-5.58956e-07,-2.42064e-09, 4.53695e-10,
	-4.34112e-11,  1.3192e-11, 1.26698e-13,-9.57036e-15,-3.29587e-14,
	 -2.6186e-06,-3.22411e-07, 1.46013e-08, 1.58574e-09, 1.18892e-10,
	 -1.4055e-12,-1.07805e-12,-7.35135e-14,-4.24723e-14,-1.20089e-15,
	   2.271e-06, 1.14011e-07,  1.9732e-08, 2.25397e-09, -7.8516e-12,
	 2.30961e-12,-3.01565e-13,-8.87665e-14,-1.46373e-14,-1.07573e-15,
	-1.19623e-16, 5.32327e-07, 2.28048e-08, 1.44595e-08,-1.17044e-09,
	-2.23265e-11,-4.47591e-12, 1.99795e-13, -5.4957e-15,-4.08906e-15,
	-3.34212e-16,-1.29188e-17, 1.73825e-20,-1.76199e-07, 1.04723e-07,
	 1.06067e-09, 8.09179e-10,-1.15208e-11,-7.18834e-13, 1.86925e-13,
	-2.27057e-14,-1.04325e-15,-2.87213e-16,-1.25304e-17,-8.85105e-19,
	-8.50245e-20 };

		const static double anm_bw_A0[91] =
		{ 0.00136127,-9.53197e-06,-6.76729e-06,-3.21103e-05,-7.84639e-07,
	 2.19477e-06, 2.18469e-05, 2.85579e-06, 2.81729e-06,  1.3181e-07,
	-1.29252e-05, 1.81685e-06, 1.63478e-06,-2.13091e-07,-2.16074e-09,
	 1.38048e-05, 7.08176e-07, 1.90932e-07,-5.37009e-08,-6.83407e-08,
	 3.96129e-09, 1.75859e-05,-1.06451e-07,-2.72757e-07, 2.92966e-08,
	 -1.2206e-08, 1.63224e-09, 3.86319e-10,-1.23856e-05, 2.99866e-07,
	-5.05748e-07,-2.66062e-08, 5.30855e-10, 6.03737e-10, 1.78082e-10,
	 9.35003e-12,-1.31216e-05,-1.23108e-06,-2.29647e-07,-2.73845e-08,
	-2.68863e-11, 1.98303e-10, 9.48294e-12,-2.25188e-11, 1.29532e-12,
	-2.45864e-06,-9.27805e-07,-1.39624e-08,-9.52539e-09, 3.79767e-09,
	 5.08814e-11, 8.95094e-12,-6.47182e-12,-1.25291e-12, 3.73573e-14,
	 8.95484e-06,-4.59185e-07,   1.634e-07, 1.22887e-08, 2.51749e-09,
	 4.85671e-11,  4.1139e-12,-1.47615e-12,-1.02258e-13,-3.19081e-14,
	-5.87702e-15, 1.86187e-06,-5.07077e-07,-7.38992e-08,-1.99777e-09,
	-7.05789e-11,   1.103e-11,-6.47394e-12,-1.23958e-13,-2.35479e-14,
	 1.03992e-14,-1.09012e-15,-1.38172e-16, 1.92989e-06, 6.49034e-08,
	-2.91989e-08, 7.31079e-09, 7.06656e-11,-2.52394e-11, -4.1567e-12,
	 3.07891e-13, 7.02157e-15, -1.7511e-15, 1.16039e-16, 5.72659e-18,
	 1.03352e-17 };
		const static double anm_bw_A1[91] =
		{ -6.83476e-07, 6.58704e-06, 6.03831e-07,-7.88685e-06, 3.11625e-06,
	-2.20581e-07,-1.80674e-06, 1.57201e-07, 3.02487e-07, -1.5829e-07,
	-1.07247e-05,-1.17362e-06,-2.68585e-08, 4.05837e-08, 9.37632e-09,
	 2.05528e-05,-9.27988e-07,-7.44215e-08,  2.2287e-08,-1.95047e-10,
	-6.63809e-10,-2.81238e-06, 1.54337e-06, 1.71139e-07,-2.07557e-08,
	 6.52492e-09, 4.95586e-10,-2.06467e-10, 7.61047e-06, 1.87526e-07,
	 1.95524e-08, -5.4047e-09, 5.28852e-10,-3.52191e-12, 2.93772e-12,
	-8.23134e-13,-5.70452e-06, 5.53093e-07,-5.45267e-08,-4.48345e-10,
	-2.20547e-09, 6.04782e-11,-3.05874e-13,-3.91628e-13, 1.66337e-13,
	 4.49461e-06,-4.97734e-07, -1.1258e-07,-5.40909e-09,  1.1155e-10,
	 1.43251e-11, -3.2622e-12, 1.13776e-12,   8.635e-14,-5.37809e-16,
	-3.90778e-06, 1.09498e-07, 1.50909e-08,-4.06647e-10, 4.79896e-10,
	-2.51743e-11,-8.09433e-13, 4.66726e-13, 1.10394e-13,-9.53904e-15,
	-1.80134e-15, 8.33098e-06, -8.6196e-07,-8.39473e-08,-3.21265e-09,
	 1.21841e-10,-3.22958e-11, 1.03157e-12, 2.85997e-13, 1.97847e-14,
	 1.54017e-15,-6.04168e-16,-6.33038e-17,-3.82662e-06, 2.41396e-07,
	-6.03828e-09,-8.42775e-10, 1.43441e-10, 1.07521e-11,-1.95963e-13,
	 1.91523e-13, 9.52231e-15, 5.29265e-16,-6.69644e-17, 1.30358e-18,
	-2.30634e-18 };
		const static double anm_bw_B1[91] =
		{ -1.37212e-06, 2.42001e-06,-8.72569e-08,-2.55496e-06,  9.0217e-07,
	 6.94761e-07,-1.52754e-06,-2.80599e-07,-1.64836e-07,-7.05581e-08,
	 -3.3111e-06,-3.19205e-08, 4.94662e-09, 2.94496e-08,-2.05996e-08,
	 1.59072e-05, 1.60536e-07, 1.81331e-08,  1.7374e-08, 2.57536e-09,
	 2.44781e-10,-2.43671e-06, -2.0069e-07, -2.5508e-08, 5.45985e-09,
	-1.79332e-10,-3.07879e-10, 4.37331e-11,-1.99104e-07, 4.99375e-08,
	-9.17535e-08,-2.29719e-09,-2.22279e-10,-1.30372e-10,-1.04698e-11,
	 5.54878e-13,-5.11599e-06, 8.60093e-07, 2.85298e-08,-1.93967e-09,
	 6.56117e-10, 2.91291e-11, 5.31539e-13,-6.80217e-13, 3.25543e-13,
	  1.0389e-06,  9.1868e-08, 2.61821e-08, 1.88699e-09,-6.45505e-10,
	 8.81617e-12,-1.28048e-12, 2.84856e-13,-1.21308e-13, -4.2343e-16,
	-1.12115e-06, 1.31664e-07, -1.1148e-08, -6.4912e-10,-2.44908e-10,
	-2.60819e-11,-1.19812e-12,  1.7209e-13,-3.14154e-15, 2.74543e-15,
	-3.09793e-16, 3.18278e-06, -3.5169e-07,-2.51587e-08,-4.84477e-10,
	 7.97348e-11,-1.43536e-11,  -9.201e-14,  1.9158e-13, 1.58936e-15,
	  1.0595e-16,-9.10902e-17,-1.38778e-17, -4.6019e-07, 2.73949e-07,
	 1.18462e-08, 1.10191e-09, 1.46962e-10,  5.8467e-12, 2.16978e-13,
	 1.05367e-13,-3.41019e-15, 4.45016e-16, 2.65667e-17, 2.49193e-18,
	 2.84778e-18 };
		const static double anm_bw_A2[91] =
		{ 7.02562e-07,-6.04283e-07,  2.2175e-06,-1.99602e-06, 6.37067e-07,
	 4.80771e-07,-3.51212e-07,-4.91267e-07,-2.11608e-07, 5.56781e-08,
	 2.13777e-06, 4.09166e-07,-7.09028e-08,-1.75005e-08, 6.97068e-09,
	 7.34089e-07,-3.27297e-07,  4.3715e-08,-4.45137e-09, 4.82643e-09,
	 5.92281e-11, 3.58245e-06,  1.3879e-09,-8.40793e-09, 8.76858e-09,
	 3.75922e-10,-7.78354e-11,-5.00421e-11, 6.86178e-07,  4.8623e-07,
	-2.56672e-08,-3.73329e-09,-5.52629e-11,-9.12787e-12, -2.8226e-11,
	-3.62943e-13,-4.99991e-06,  2.6557e-07, 1.98168e-08,-1.41509e-09,
	  1.2713e-10, 6.18099e-11,-7.31016e-12, 5.91034e-13, 1.89143e-13,
	-2.73783e-06,-2.47201e-07,-2.31876e-08,-4.08128e-09, 3.05477e-10,
	 2.58969e-11, 2.67562e-12,-7.63667e-14, 5.12571e-14,-4.75111e-15,
	 1.78679e-06, 4.20526e-08,-1.84001e-08, 8.63651e-10, 2.56736e-10,
	 6.12728e-12, 1.46886e-12, 1.13855e-13, 2.41103e-14, 2.33798e-15,
	-1.04198e-16,-7.68274e-07,-4.20797e-07, 7.30691e-09,-1.82925e-09,
	 1.08276e-10, 6.87097e-12, -2.4015e-13, 5.20597e-14,-2.32217e-15,
	-1.17871e-15, 1.98379e-16, 1.94191e-17, 2.24291e-06, 1.10634e-07,
	 1.32096e-08, 1.96592e-09, 7.44593e-11,  6.5203e-12,-2.84701e-13,
	-5.28136e-14,-3.59825e-15, -3.8018e-16, -9.7601e-17, 1.76247e-18,
	-7.72161e-19 };
		const static double anm_bw_B2[91] =
		{ -2.16342e-07, 2.02144e-07, 1.05146e-06,-4.62005e-07,-9.44485e-09,
	-1.34358e-07, 2.73741e-06,-2.11648e-07,-6.47818e-08, 1.23403e-08,
	-1.49519e-07, 1.53287e-07, 4.44353e-08,-3.03016e-09,-8.90989e-09,
	 1.28227e-06,-2.20518e-07, 4.18884e-08, 9.44722e-09, 3.37657e-09,
	-4.78503e-10,-1.76547e-06,-1.62491e-07,-1.01129e-08, 1.06786e-08,
	-7.06417e-10, 1.43959e-10,-9.40238e-12,-1.09467e-07, 4.46571e-07,
	-7.11896e-08, 3.38748e-09,-9.24825e-10, 6.42187e-12, -5.6181e-12,
	 2.38859e-12, 1.27389e-07, 1.95485e-07, 5.52227e-09,-1.75457e-09,
	-2.32335e-10,-3.82682e-11,-1.19921e-11, 5.02992e-13, 3.32288e-13,
	 7.12696e-07, 6.16164e-08,  5.7268e-08, 1.09535e-10, 1.26261e-10,
	 3.83422e-11, 2.72195e-12,-1.34452e-13, 3.32389e-14, 6.02553e-15,
	 1.46806e-06,-9.71471e-08, 7.82125e-09, -2.7344e-09,-1.64815e-10,
	 2.16833e-11, 3.15808e-12, 2.77741e-13, 2.13853e-14,-2.53192e-15,
	 3.72782e-16,-1.52337e-06,-3.07653e-07,-3.19457e-08,-3.46664e-10,
	-1.17891e-10,-6.64963e-12, 1.14009e-12,-4.06741e-14,-8.48612e-15,
	-1.10937e-15,-1.03715e-16, 5.70056e-18, 1.40544e-06,-3.19555e-08,
	-5.06573e-09,-2.13136e-09,-3.64332e-11, 1.82947e-12, 4.27195e-13,
	-3.53364e-14,-2.62576e-15,-4.32918e-16, 4.07313e-17, 7.59614e-19,
	 6.07028e-19 };

		const static double anm_ch_A0[91] =
		{ 0.0571481, 5.77367e-05,-8.50773e-05,  0.00150536,-2.32169e-05,
	-1.38607e-06, 6.52492e-05, -0.00013887,-9.98524e-07, 1.27066e-06,
	-0.000377265,-4.93721e-05,-2.21882e-06, 2.21876e-07,-3.51739e-08,
	 3.51241e-05, -2.3627e-05,-7.91766e-07, 3.92769e-07, -1.5665e-08,
	-2.80965e-08,-0.000638228, 4.47833e-06,-2.26958e-06,-3.98954e-07,
	 2.30419e-08, 3.69424e-09,-1.38609e-09,-4.35499e-05, 1.24888e-05,
	 9.50251e-07, 5.91775e-09,-1.78491e-08, 1.75054e-09, 1.08153e-09,
	-1.88341e-10, 0.000359778, 5.23718e-05, 1.00109e-06,-2.07014e-07,
	 2.03081e-09, 3.17469e-09, 9.89541e-11, 5.01721e-11, 2.53138e-11,
	 0.000115735,-9.76879e-06, 1.04903e-07, 7.63177e-08, 3.29856e-09,
	 3.85374e-10,-1.08841e-10,  2.0739e-11, -2.8588e-12, 6.91891e-13,
	-5.06133e-05,-1.27946e-05,-8.71907e-07,-6.23652e-08,-1.46315e-10,
	 3.31888e-10, 1.36646e-11, 9.44685e-13,-8.13097e-13,-3.76176e-14,
	-4.81393e-14,-7.81077e-05,  5.8914e-06,-1.44273e-06, 3.98889e-08,
	 1.60727e-09,-2.55708e-10,-3.35886e-11,-3.12789e-12, 6.83792e-14,
	-1.28585e-14,-9.35623e-15, 8.79757e-16, 0.000117829,-1.87058e-05,
	 2.79836e-07, 4.95854e-08, 4.71498e-09,-1.07341e-10, -3.8313e-11,
	-5.64389e-13, 1.78189e-13,-8.18481e-15, 6.16579e-15, 9.35482e-17,
	 2.34173e-17 };
		const static double anm_ch_A1[91] =
		{ 3.35402e-05,-0.000669057,-4.87107e-05, 0.000611682,-0.000127479,
	-3.97272e-06, -0.00112224,-7.29996e-05,-6.84967e-08, 1.12113e-06,
	 0.000262691, -0.00010282,-1.40207e-06, 1.92273e-07, 2.89616e-08,
	-0.000725895,-4.20692e-05, 1.64557e-06,-1.98028e-07, 7.61808e-08,
	-7.25951e-09,-0.000154814, 1.33712e-05,  2.5778e-06,-2.26931e-07,
	 1.23099e-08, 3.49231e-10,-1.86719e-10, 0.000450728, 1.34275e-05,
	 2.34367e-06,-1.48971e-07, 1.76865e-08,-1.31414e-09,-3.85678e-10,
	 5.19645e-11,-0.000111693, 1.32279e-05,-2.75515e-07,-7.31392e-08,
	-1.35424e-08, 2.44221e-10,-2.78763e-11,  1.6647e-11,  1.9303e-12,
	 0.000116911, 9.16968e-06, 2.14037e-07, 1.22782e-07,-6.95004e-09,
	 1.65754e-10,-4.56043e-11, 1.51187e-11, 1.92164e-13,-8.49442e-14,
	-4.28349e-05,-1.92363e-05,-1.02092e-06, 6.98993e-08,-7.85143e-09,
	 1.90932e-10, -5.6883e-12, 7.30314e-12, 5.23352e-13,-6.19954e-14,
	 3.66085e-15,-0.000254774,-3.17313e-06,-1.10237e-07,-1.29638e-08,
	-2.70751e-09, 2.27878e-11,-3.51895e-12, 3.35367e-12, 2.41583e-13,
	 -2.1105e-14,-3.94385e-16,  4.5253e-16, 2.67014e-05, 1.34371e-05,
	 8.93093e-07,-1.44695e-08,  2.4751e-09,-8.07692e-11,  -5.825e-12,
	 9.57258e-13, 7.82582e-14,-3.71257e-15, 6.74087e-16, 6.23536e-17,
	-8.30342e-18 };
		const static double anm_ch_B1[91] =
		{ 3.15988e-05,-6.51058e-05, 4.03431e-05,  0.00036973,-9.00292e-05,
	-8.16155e-07,-0.000344968, 5.35987e-05, 1.47478e-07, 2.71526e-07,
	  0.00018364,-5.69904e-05, 1.60957e-07,-6.44017e-09,-3.40343e-08,
	-0.000322514, 1.43927e-05, -9.4393e-07,-5.41304e-08,  -1.819e-08,
	 -2.5979e-09, 7.78518e-05, 3.61049e-06, 1.08396e-06, -1.0917e-07,
	-1.71161e-08,-1.18581e-09,-8.13477e-10, 0.000328978, 1.11276e-06,
	-5.43099e-07,  2.9984e-08, -1.9674e-08,-1.44689e-09,-2.77851e-10,
	 2.14131e-11,-6.87103e-05,-5.72277e-07,-1.13393e-06,-3.96446e-08,
	-4.47637e-09,-2.46821e-10,-2.10948e-10,-7.50625e-12,-1.44709e-12,
	 8.36316e-05,-8.72755e-06,-7.19359e-07, 8.99971e-08,-6.52491e-09,
	-3.31326e-10,-4.77254e-12,  9.2814e-12, 4.02665e-14,-5.54405e-15,
	-6.90747e-05,-2.30972e-06,-1.69548e-07, 5.91598e-08, 1.48788e-09,
	-3.13672e-11, 1.57519e-11, 3.14451e-12, 8.94349e-14,-3.74537e-14,
	 3.10432e-15,-0.000214539,-3.81873e-06, 2.64337e-07,-4.13668e-08,
	 2.79387e-09,-6.43063e-12, 5.80698e-12, 2.52237e-12, 8.58808e-14,
	  5.2641e-15, 5.04633e-16, 1.36886e-16, 2.57913e-05, 5.59131e-06,
	 8.37295e-07, 2.51662e-08, 1.67324e-09,-5.99382e-11, 1.43286e-11,
	 1.12055e-12, 5.18062e-14, 4.23509e-16, 5.71629e-16, 3.18077e-18,
	-4.33602e-18 };
		const static double anm_ch_A2[91] =
		{ -1.34477e-05, -1.6183e-06,  2.5409e-06,-1.95658e-05,-6.07834e-05,
	 5.73267e-07,-7.67283e-05, 9.03805e-06, 1.94858e-06,-2.76126e-07,
	 3.93177e-06,-3.79603e-05, 2.06121e-06,-1.40955e-07,-2.89763e-08,
	-0.000106144, 2.61811e-05, 6.46642e-07,-4.21706e-07, 1.30196e-08,
	-2.79272e-09,-2.95961e-05,-2.50718e-06, 1.29403e-07, -1.4951e-07,
	  2.3583e-09, 5.43181e-10,  2.0192e-10,-3.05249e-05, 7.46734e-06,
	-4.35197e-07,-1.30596e-07, 1.21097e-08, 1.14682e-09, 3.89891e-11,
	 1.24028e-11, 0.000115129,-7.56327e-06, -4.7505e-07,  3.2139e-08,
	 2.18409e-09, 7.55851e-10, 3.77882e-11,  9.9788e-12,-1.77837e-12,
	 1.61096e-05,-1.30078e-05,  1.1253e-07, 5.63482e-08, 1.97749e-09,
	 5.93256e-10,-2.94405e-12, 5.92738e-12, 5.18246e-13, 9.71093e-15,
	  8.4838e-05,-8.98516e-06, 4.87239e-07, 4.36227e-08,  2.1687e-09,
	 2.60712e-10,-1.61936e-11,  6.4903e-13,-1.33328e-13, 1.71275e-14,
	-4.10964e-15,-3.80781e-05,-2.27592e-06, 4.76074e-07, 1.27687e-09,
	 -3.0142e-10, 1.26532e-10,-2.84881e-12,-8.12801e-13,-5.12528e-14,
	-4.31737e-15,-5.40269e-16,-1.12984e-16,-4.40766e-05,  4.5096e-06,
	 -1.9903e-08,-3.87087e-09, 8.14197e-10, 2.33174e-12, 3.15151e-12,
	-4.42418e-13, 2.13457e-14,-2.91293e-15,-2.05251e-16,-2.92353e-17,
	 1.90226e-18 };
		const static double anm_ch_B2[91] =
		{ -2.61831e-07, 8.96771e-05,-5.59109e-06,-3.46247e-05,-1.04628e-05,
	 2.00366e-07, 0.000107907,  8.6137e-06, 7.17177e-07, -1.0543e-07,
	-6.66187e-06,-3.96726e-06, 6.50945e-07,-4.26742e-07,-6.40981e-10,
	 4.08153e-05, 2.09611e-05, -5.9151e-07,-6.06042e-08, 1.08616e-08,
	 4.10559e-09, 1.15965e-06,-1.28101e-05,-1.04855e-06,-3.98377e-07,
	 1.31136e-08, 5.43192e-10, 1.00068e-10,-3.21915e-05,-2.12459e-06,
	-8.31852e-07, 1.27137e-07, 2.95519e-10, 1.74489e-09,-2.54627e-10,
	-9.42819e-12, 7.59796e-05, -1.9575e-05,-3.21499e-07,-1.43739e-08,
	-3.74845e-09, 6.98981e-10, -1.2001e-12, 4.87142e-12,-8.10069e-13,
	-7.53085e-05,-9.78842e-06, 7.07316e-07, 4.31054e-08, 3.51942e-09,
	 1.27725e-10,  -3.072e-11, 9.70337e-13,-7.91439e-13,-5.33714e-14,
	 7.04136e-05, 5.25675e-06, 9.13163e-07, 6.45322e-08,-1.16723e-09,
	  8.4524e-11,-4.16568e-12,-9.66911e-13,-4.04549e-13,-3.81947e-14,
	-6.58644e-15, 1.83495e-05, 1.57045e-06, 4.28624e-07,-3.54203e-08,
	-1.29102e-10,-1.65822e-10, 8.91708e-12, -2.6351e-13,-1.40962e-13,
	-1.60992e-14,-1.07858e-15, 6.30355e-18,-1.60652e-06, 2.87613e-06,
	 -8.8724e-08, 2.29742e-08,-3.71467e-10,-2.26994e-11, 5.97891e-12,
	 -9.9319e-13,-5.26202e-14,-1.15454e-14,-7.25999e-16, 7.65216e-19,
	-7.85508e-19 };

		const static double anm_cw_A0[91] =
		{ 0.039533,-0.000441838, 1.30419e-05, 3.68918e-05,-6.50227e-06,
	 2.67879e-05, 0.000393938, 5.00215e-05, 3.91965e-05, 6.68159e-06,
	 0.000314197, 0.000104598, 8.95483e-06, -1.3525e-05,-1.68143e-06,
	-0.000639889, 2.77063e-05,-2.69398e-06,-2.63719e-06,-1.56635e-06,
	-2.37494e-07,  0.00134012,-0.000113287,-4.97413e-07, 4.81989e-06,
	  1.1014e-08,  6.1242e-08,  9.1709e-10,-0.000424913, 3.85435e-05,
	-7.38204e-06, 5.89841e-07, 4.85165e-08, 1.13056e-08, 3.16454e-09,
	 1.27573e-09,-0.000481656,-1.52987e-05,-8.65401e-06,-1.68156e-06,
	-1.08049e-08,-5.98864e-09,-7.33114e-10, -5.2334e-10, 3.75348e-11,
	-0.000686805, 3.35231e-05, 3.60564e-07,-4.09605e-07, 6.74886e-08,
	 1.31707e-09,-3.68427e-11, -7.9003e-11,-2.36147e-11,-3.30107e-13,
	 0.000430216, 1.66861e-05, 1.53552e-06, 2.56647e-07, 1.77744e-08,
	 1.61133e-09, 1.23344e-10,-6.55745e-12,  -2.892e-12,-1.61679e-13,
	-1.16808e-13,-0.000113717,-2.13147e-06,-2.88001e-06,-1.10761e-07,
	-9.56994e-09, 4.62559e-10,-7.23385e-11, -1.6119e-11, -8.7223e-13,
	 7.99312e-15, 4.80504e-14,-1.20465e-15, 1.68529e-05,-3.28152e-05,
	-2.08665e-06,-2.88206e-09,-7.09711e-09, -8.1191e-10, 1.98628e-11,
	 2.62289e-12,-3.37805e-13,-1.85034e-13, 2.96114e-15, 1.40075e-15,
	 2.96823e-16 };
		const static double anm_cw_A1[91] =
		{ -0.000131114, 0.000701289,-0.000185047,-0.000219433, 2.07615e-05,
	-6.89304e-05,-0.000452948,  3.5723e-05, 1.02237e-05,-8.10469e-06,
	-0.000296548,   -3.31e-05, 1.44019e-05, -2.4935e-06, 1.72073e-06,
	  0.00120194,-9.51425e-06, 6.50861e-06, 5.03149e-07,-2.97026e-08,
	 1.69782e-08,-0.000250989, 4.78919e-05,   3.726e-06,   4.214e-07,
	 1.90258e-07,-4.48953e-09,-9.24879e-09,-0.000114365,-5.70727e-05,
	-4.56639e-06, 1.40863e-08, 8.34274e-08, 3.50569e-09,-1.39635e-09,
	-3.51626e-10,-0.000245423, 2.78384e-05,-6.51883e-06,-3.53603e-07,
	-9.78851e-08, 7.38971e-09,-9.08374e-11, 1.36404e-10,-1.08518e-11,
	  0.00059185, 2.38332e-05,  -3.133e-06,-2.51159e-07,-1.02009e-08,
	-1.06244e-09,-1.90028e-10, 3.81214e-11, 1.03853e-11,-4.73989e-13,
	-0.000132831,-8.10093e-06, 2.19486e-06,-8.07492e-08, 1.64135e-08,
	-1.85415e-09,-1.20385e-10, 1.22127e-11, 5.05963e-12,-3.92206e-13,
	-9.97465e-14,-0.000131903,-3.57819e-05,-2.09957e-06, 6.91774e-08,
	 6.64105e-09,-1.69015e-09, 5.51961e-11, 1.66868e-11, 1.88614e-12,
	 4.46151e-14,-3.35546e-14,-8.80752e-16, 3.57144e-05, 1.04232e-05,
	-1.64464e-06,  2.3832e-09, 6.29471e-09, 3.34048e-10,-4.98558e-12,
	 3.91242e-12, 5.39455e-13, 4.31132e-14,-1.11689e-14,-5.15564e-16,
	 -4.7703e-17 };
		const static double anm_cw_B1[91] =
		{ -0.000116331,  0.00033849, 4.31032e-05, 3.46769e-06,-5.09131e-05,
	 2.11047e-06,-0.000136518,-9.38011e-07,-1.95711e-05,-9.91193e-06,
	 -0.00021841, 5.60327e-05, 4.87913e-06,-2.51509e-06, 1.38766e-06,
	 0.000753259,-6.61068e-06, 3.80856e-06, 7.38965e-07, 5.06603e-08,
	 8.35178e-08,-2.97648e-05,-3.14572e-05,-3.53222e-06, 1.02815e-06,
	-1.00831e-08,-1.38838e-08,-2.30857e-09,-0.000403201, 4.96314e-07,
	-3.94008e-06, 1.08149e-07,  1.4749e-08,-5.09396e-09,-7.34085e-10,
	-4.84132e-10,  -0.0002395, 4.32299e-05, -2.4381e-07,-1.00404e-07,
	-2.32907e-08, 3.61323e-09,-1.78204e-10,-7.04479e-11,-2.53584e-12,
	 0.000475117, 7.43546e-06,-6.38975e-07,-1.29359e-07, -3.3079e-08,
	-2.85574e-09,-3.98587e-11, 4.63303e-11,-4.47242e-12,   5.172e-13,
	-3.41831e-05, 8.20658e-06,-1.07254e-06,-2.05858e-07, 4.86163e-09,
	-2.13799e-09,-2.18038e-12, 5.83805e-12, 1.28093e-12,-9.04941e-14,
	 7.46367e-15,-0.000162845,-1.50826e-05, 6.81467e-08, 8.17125e-08,
	 6.33078e-09, -2.8026e-10, 3.04071e-11,  1.0537e-11, 1.21711e-14,
	 7.50407e-14,-1.18546e-14,-1.22214e-15,   -1.66e-05,-2.80762e-06,
	 6.79099e-08, 1.14169e-07, 3.50833e-09, 3.70619e-10,-5.90523e-12,
	 6.56035e-12, -2.4157e-13, 4.13497e-15, 3.76275e-15, 3.56287e-16,
	 1.12782e-16 };
		const static double anm_cw_A2[91] =
		{ 6.23548e-05,   3.767e-05, 0.000105583,-9.17185e-05,-3.08053e-05,
	 1.93164e-05, 0.000138239,-3.52587e-05,-5.93905e-06,-1.89756e-07,
	-1.57318e-05, 4.71895e-05,-3.45826e-06,-9.14255e-07, 4.09771e-07,
	 3.87356e-05,-1.38714e-05,-1.98076e-06,-8.38892e-07,-4.60436e-08,
	-4.83564e-08, -6.4789e-05,-2.10518e-05,-1.05232e-06, 4.40299e-09,
	 1.44861e-08,-2.05534e-08,-4.36349e-09,  4.1995e-05, 1.02396e-05,
	-2.16667e-06,-2.18593e-07, 6.01494e-08,-1.83362e-09,-7.55541e-10,
	 1.52427e-10,-0.000157133, 1.70981e-05, 8.54349e-07,-2.14162e-08,
	 2.22004e-08, 3.70037e-09, 8.28618e-11,-6.83175e-12, 1.00168e-11,
	-2.59339e-05,-3.41431e-06,-8.63986e-07,-2.27745e-07,  1.6496e-08,
	 3.72567e-09, 1.14459e-11, 1.52295e-11, 5.30884e-12, 2.34952e-13,
	 4.70312e-06, 6.12399e-06,-4.72142e-07, 1.09784e-07, 1.13334e-08,
	 1.15255e-09, 3.23033e-11,-8.31202e-12, 5.60354e-13, 1.89848e-13,
	 2.53399e-14, 7.90172e-06,-2.17909e-05, 3.58309e-07,-2.16476e-08,
	 2.85721e-09, 4.27559e-11, 3.23227e-12,-4.41496e-12,-1.13342e-12,
	-3.20385e-14, 4.19419e-15, 1.69929e-15,  5.4037e-05, 2.98996e-06,
	 7.23956e-07, 8.12981e-08, 8.31289e-09, 3.30426e-10,-1.27027e-12,
	-1.17413e-12,-2.41572e-13,-4.60076e-14,-3.72593e-15, 5.07243e-16,
	 1.58443e-18 };
		const static double anm_cw_B2[91] =
		{ 5.72641e-05,-8.70889e-06, 3.23045e-05,-3.69243e-05,-4.18484e-05,
	-1.97877e-06, 0.000133175,-7.01219e-06,  3.2434e-06,-3.26799e-06,
	  4.6979e-05, 3.57432e-05,  3.2396e-06,-8.57897e-07,-6.60909e-07,
	 1.31232e-05, 9.84662e-06, 1.17187e-06, 1.30944e-06,-1.62536e-07,
	-4.96449e-08, 8.41302e-05,-2.03934e-05,-2.74821e-06, 3.37103e-09,
	-5.29882e-08, 1.49518e-09,-4.45809e-10,-3.02068e-05,  5.8555e-06,
	-4.55694e-06,-3.78927e-07, 6.43813e-09,-4.11227e-09,-1.57569e-10,
	 1.27466e-10, 5.54583e-05,-1.35091e-06, 2.98372e-07,-2.42132e-07,
	 -2.2723e-09,-3.41121e-09,-1.32967e-10,-2.86041e-12,  1.7493e-11,
	 3.74825e-05, 7.20181e-06, 2.43368e-06, 7.04066e-08, 1.40642e-08,
	 2.48403e-09,-2.27722e-12,-2.99095e-12, 1.68499e-12, 2.05931e-13,
	-3.84807e-05,-1.85536e-06, 4.00745e-07,-7.76823e-08,-7.17261e-09,
	 2.24505e-09,  8.0118e-11, 1.90985e-12, 1.76754e-12, 4.10009e-14,
	 1.06511e-14,-0.000178768,-8.19332e-06,-4.18502e-07, 7.59222e-08,
	-6.39667e-09,-1.66926e-10, 8.47312e-11,-2.24939e-12,-6.87152e-13,
	-1.26544e-14,-1.73526e-14, 1.93593e-16, 0.000118138,-2.67641e-06,
	-6.86378e-07,-1.56958e-07,-2.14221e-09, 4.86297e-11, 1.49982e-11,
	-9.40878e-13,-3.01984e-13,-1.92455e-14, 1.98205e-16,-2.30406e-17,
	 8.22142e-17 };

		///
		const static double bnm_bh_A0[91] =
		{ 0,           0,-2.29211e-06,           0,-4.63383e-06,
	-6.92432e-07,           0,-1.91508e-06,-8.07527e-07,-2.82995e-07,
			   0, -6.6821e-07,-2.36947e-07, 1.07746e-07,-5.44712e-08,
			   0,-5.61707e-08, 2.41783e-08, 6.38293e-08, 1.09291e-08,
	-4.40958e-09,           0, 1.54754e-08, 1.37131e-08, 2.19113e-08,
	 6.26573e-09, 1.65048e-11, 6.20222e-11,           0,-1.03939e-06,
	-4.36557e-08, 3.43194e-10,-4.73308e-10,-3.36186e-10, 3.19641e-11,
	 1.68929e-11,           0,  2.3295e-07, 8.89573e-08,-1.00824e-08,
	  1.1326e-09,-2.37748e-10,-1.54978e-11, 3.28286e-12, 4.57597e-13,
			   0,-3.67821e-07, 5.17794e-08,-7.94293e-09,  1.3641e-11,
	 -1.4162e-11,-3.56522e-12, 2.22759e-12,-6.26834e-14, 1.81973e-14,
			   0, 9.74322e-07,-6.75813e-09, 4.53859e-10,-4.92901e-10,
	 1.38469e-11, 4.02248e-12, 4.15006e-13, 6.36377e-14,-5.55274e-15,
	 1.00936e-17,           0, 4.21959e-07, 2.64389e-08, 6.31204e-09,
	 1.18415e-10,-4.20826e-12,-9.47668e-13, 2.23948e-13,-3.88008e-16,
	-8.62166e-15,-2.35355e-16,  6.0191e-17,           0,-3.75006e-07,
	 7.82607e-09, 4.28818e-09, 1.28984e-10,-4.48914e-12,-1.12627e-13,
	 5.08294e-14,-4.01464e-16,-1.73919e-15,-4.61556e-17,-2.27041e-17,
	 5.67242e-18 };
		const static double bnm_bh_A1[91] =
		{ 0,           0,-2.33805e-06,           0,-2.23853e-06,
	-2.98734e-07,           0,-1.83615e-06, 2.83998e-08, 1.38818e-09,
			   0,-2.19105e-06,-1.32731e-07,-4.17089e-09, 1.89717e-09,
			   0,-1.09066e-06,-3.65762e-08, 1.11378e-08, 1.23936e-10,
	 1.45751e-10,           0,-1.60155e-06,-6.67814e-08, 4.11027e-09,
	 5.17137e-10, 1.00047e-10, 9.75853e-12,           0,-1.78417e-07,
	  7.4978e-09, 1.13545e-08, 1.70856e-10, 3.19243e-11, 2.94298e-12,
	-7.90033e-13,           0, 1.46238e-07,  7.2481e-08, 8.84434e-10,
	 5.55187e-10, 2.45464e-11, 4.58755e-12,-1.10072e-12, 9.84198e-14,
			   0, 6.98498e-07, 5.54614e-08,-1.01709e-09,  2.1639e-10,
	-1.54733e-11, 4.47877e-12,-4.63876e-13, 2.73416e-14, 5.43036e-15,
			   0,-5.23102e-07, 3.49745e-08,-1.49347e-09, 5.34223e-12,
	-7.91181e-12, 1.82777e-12,-2.00648e-13, 1.65795e-14, 3.69725e-15,
	 6.54752e-17,           0,-8.30678e-08, 2.65825e-09,-1.67038e-09,
	-2.57745e-11,-6.97431e-12, 6.53305e-13, 7.35263e-15, 2.28561e-15,
	 1.79743e-15,-6.13998e-17,  5.3552e-18,           0,-5.39872e-07,
	-1.68891e-08,-1.07501e-09,-6.96375e-11, 9.68954e-13, 3.34903e-14,
	 2.83036e-14, 1.97048e-15, 7.52792e-16,  4.9453e-19,-3.27856e-18,
	 3.84449e-19 };
		const static double bnm_bh_B1[91] =
		{ 0,           0,-7.49313e-07,           0, 8.14831e-07,
	 1.48085e-08,           0,-7.46807e-07, -6.0189e-08, 3.22731e-09,
			   0, 1.30117e-07,  1.8367e-08,-1.83296e-09, 2.27327e-10,
			   0,-2.25742e-07,-6.93421e-08, 6.91425e-09, 3.92917e-10,
	 1.24641e-10,           0,-4.08425e-07,-7.01411e-09, 6.33816e-09,
	-3.49784e-10, 1.05125e-10,-5.46716e-12,           0,  2.8604e-07,
	-8.62829e-09, 1.25242e-09, -2.6247e-11, 1.17589e-10,-1.00652e-11,
	  -1.409e-12,           0,-1.07771e-07, 2.67584e-08,-2.55503e-10,
	  7.6368e-11, 3.23208e-11,-1.25865e-12,-7.23471e-13, 3.34504e-14,
			   0, 1.83397e-07, 1.75026e-09,-1.49251e-09, 1.24935e-10,
	 1.36793e-11,-6.34096e-16,-5.80688e-13, 1.91842e-14, 1.92477e-15,
			   0,-2.81998e-07, -5.0917e-09, -2.0094e-09, 1.08502e-10,
	 2.26642e-13,-1.28349e-13,-1.67278e-13,-3.80833e-15, 2.02114e-15,
	-1.09139e-16,           0,-3.47006e-07,-1.28896e-08, 1.33105e-09,
	 2.88296e-11,-3.90661e-12,  2.6324e-13,-3.83348e-14,  -8.736e-16,
	 1.01541e-15,-2.73491e-17, -2.1553e-18,           0, -1.1993e-07,
	-8.45995e-09, 8.84086e-10,-2.13481e-11,-1.61372e-12,-1.21722e-13,
	 1.48075e-14,-5.29968e-16, 3.65589e-16,-1.09858e-17,-3.30649e-19,
	 1.77668e-19 };
		const static double bnm_bh_A2[91] =
		{ 0,           0,-5.12023e-07,           0, 1.15453e-06,
	 1.37882e-07,           0,-1.28329e-06, -2.4834e-08, 2.87731e-10,
			   0, 4.78446e-07, 7.90219e-08,-5.80244e-09, 7.78401e-10,
			   0,-8.64368e-07,-3.97316e-08, 1.39887e-09,-1.79145e-10,
	 -6.4581e-11,           0,  6.1817e-09, 3.82733e-08,-1.49242e-09,
	 1.13579e-10,-3.03826e-11, 1.31643e-11,           0, 1.83509e-08,
	 5.50578e-09, 6.03222e-10, 2.04526e-10,-1.35479e-12,-1.67029e-12,
	 7.76938e-13,           0, 1.26825e-07, 2.48435e-08,-5.48034e-10,
	-1.70068e-11,-2.72625e-12,  2.4414e-12,  5.8531e-13, 7.08431e-15,
			   0,  2.3973e-07,-2.55518e-09, 9.32827e-10,-6.82508e-11,
	 1.11158e-12,-1.13011e-12, 2.45879e-13, 1.67405e-14, 8.78499e-17,
			   0, 4.54762e-08, 5.24359e-09, 9.30987e-10,-6.43526e-11,
	 4.55252e-12,-2.16257e-13, 1.30332e-13,-6.40263e-16,-4.50871e-16,
	-8.62397e-17,           0,-3.36443e-08,-7.07183e-10, 8.34376e-10,
	-3.27783e-11,-3.90449e-13, 1.40129e-13, 4.20377e-14, 2.14321e-15,
	-7.91028e-17, 2.63895e-17,-2.46778e-18,           0, 4.52771e-08,
	 1.42959e-09, 9.74171e-10, 1.33516e-11,-2.09684e-12, 7.46246e-14,
	-9.59634e-15, -3.5912e-16,-7.79248e-17, 3.95551e-18, 9.08749e-19,
	 2.00433e-20 };
		const static double bnm_bh_B2[91] =
		{ 0,           0, 5.88926e-07,           0,-4.53555e-07,
	-6.92492e-09,           0, 5.04937e-07, 2.46285e-08, 1.53896e-08,
			   0,-4.40344e-07,-4.70162e-08,-2.11682e-09,  8.8238e-12,
			   0, 1.06411e-08,-2.08768e-08, 5.25429e-09,-9.11803e-10,
	-8.92895e-12,           0, -2.5892e-07,-2.73382e-08,-6.14225e-10,
	 2.80415e-10, 4.57155e-11, 3.61619e-12,           0,-1.34452e-07,
	-9.46898e-09, 1.57172e-09, -1.1786e-10, 5.11192e-11, 3.03939e-12,
	  7.3254e-13,           0,-2.36346e-08,-3.55005e-09,-8.50242e-10,
	 1.57082e-10, 8.14449e-12,-1.82827e-12, 4.11318e-13, 2.79891e-14,
			   0,-2.58442e-07,-6.12273e-09,-8.24491e-10,-4.01575e-11,
	 2.08548e-11,-2.82018e-13, 1.49998e-13,-2.45269e-17,-1.42582e-15,
			   0,-3.34645e-08, 4.96664e-09,  9.7445e-11,-3.11063e-11,
	 6.05271e-12,-5.54364e-14, 1.52742e-13, 2.42577e-15,  9.6295e-17,
	-3.82788e-17,           0,  9.9074e-08, 7.10907e-09,-2.52479e-10,
	-1.05705e-11,-4.60385e-13, 3.85789e-14, 4.26446e-14, 6.38632e-16,
	-4.06505e-16,-4.47531e-18,-7.09947e-19,           0, 1.82791e-07,
	 3.21076e-09, 8.59877e-12,-1.65865e-11,-1.90097e-12, 3.68425e-13,
	-1.26231e-14, 1.69691e-16,  -8.246e-17, 3.23972e-18, 8.92198e-19,
	-2.00801e-19 };

		const static double bnm_bw_A0[91] =
		{ 0,           0,-9.56715e-06,           0,-7.17683e-06,
	-8.02661e-07,           0,-2.89641e-06,-1.06145e-06,-4.16549e-07,
			   0, 3.52325e-06,-2.06569e-07, 1.32438e-07,-1.29595e-07,
			   0, 4.06961e-06,  6.5337e-08, 1.44651e-07, 1.86771e-08,
	-1.00207e-08,           0, 3.07162e-06, 4.50766e-08, 3.77012e-08,
	 8.09496e-09, 9.37088e-11,-8.88504e-11,           0,-2.64054e-07,
	 7.41707e-08,-1.00254e-08,-2.18568e-09,-9.30451e-10, 5.33151e-11,
	 4.69917e-11,           0,-3.50306e-06, 1.44671e-07,-2.00043e-08,
	 1.23553e-09, -6.6775e-10,-4.62772e-11, 4.75582e-12, 1.66486e-12,
			   0, 9.93871e-08, 7.88329e-08,  -5.937e-10,-6.38102e-11,
	 1.50944e-11,-1.18165e-11, 6.01892e-12,-1.50792e-13,-1.85619e-14,
			   0,-8.16252e-08, 7.68685e-09, 7.97903e-09,-1.67853e-09,
	 4.08807e-11, 6.48923e-12, 1.28604e-12,  1.4867e-13,-1.67624e-14,
	 7.85453e-16,           0,  2.8671e-06, 2.11824e-09, 1.21757e-08,
	-9.98414e-11, -1.5727e-11,-1.20659e-12, 3.80559e-13,-4.69629e-14,
	-1.88759e-14,-9.80871e-16, 2.49371e-16,           0,-1.69222e-06,
	 1.67657e-08,  1.0608e-08,   1.624e-10, -2.0218e-11,-9.88409e-13,
	 1.71431e-13,-1.74185e-15,-6.02936e-15,-4.65633e-16,-6.25921e-17,
	 9.39427e-18 };
		const static double bnm_bw_A1[91] =
		{ 0,           0,-3.68041e-08,           0, 2.89994e-06,
	 3.66739e-07,           0,-7.83566e-07,  1.5771e-07,-1.35221e-07,
			   0,-2.26706e-07, -9.3626e-08,  4.4435e-08,-1.40853e-08,
			   0,-1.97899e-06,-2.42819e-07, 8.50666e-09,-6.44198e-10,
	 1.33934e-09,           0, 1.82962e-06,-1.67752e-07,-7.23046e-09,
	-3.01977e-10, 1.00781e-09,-1.63254e-10,           0,-2.37612e-06,
	-1.64359e-08, 1.12528e-08, 6.26422e-10, 5.62176e-11, 1.07571e-10,
	-4.34516e-12,           0, 1.76085e-06,-1.41331e-08, 1.72146e-09,
	 1.42028e-09,-5.77723e-11,-7.24005e-13,-3.79782e-12, 1.08574e-12,
			   0,-1.85265e-06,  2.0481e-08,-6.57499e-10, 4.23668e-10,
	-6.11641e-11,  -9.913e-13,-8.78487e-13,-1.45235e-13, 5.64334e-14,
			   0, 1.61725e-06,-5.00941e-09, 3.73804e-09,-1.61405e-10,
	-3.74514e-11, 1.96167e-12,-4.88147e-13, 4.41636e-14, 6.59317e-15,
	 6.91252e-16,           0, -1.6818e-06, 2.02274e-08, 1.68406e-09,
	 5.38382e-10,-9.02797e-12, 7.12941e-13, 1.19678e-13, 5.59889e-15,
	 5.20976e-15, 2.50858e-17, -1.4912e-17,           0, 1.64278e-06,
	-3.77746e-08,  3.2199e-09, 3.51323e-10, 2.03305e-11,-1.47693e-13,
	 1.73951e-13,-8.18003e-16, 1.89241e-15, 3.70282e-18,-2.10533e-17,
	 9.17045e-19 };
		const static double bnm_bw_B1[91] =
		{ 0,           0, 1.27847e-07,           0,-2.97764e-07,
	-3.02881e-07,           0,-8.36667e-07, 1.04203e-07, -6.4027e-08,
			   0, 1.53836e-06,-3.55985e-08,-3.38192e-08, 1.42588e-09,
			   0, 7.21906e-08,-1.66203e-08,-4.61166e-09,-7.60457e-09,
	 1.41692e-09,           0, 1.87729e-07, 2.47845e-08, 5.22996e-09,
	-2.30105e-09, 1.28141e-10,  7.2267e-11,           0,-1.83671e-06,
	 -3.0975e-08,-2.67842e-09, 1.95007e-09, 1.01023e-10,-1.31715e-11,
	-6.61054e-12,           0, 8.16661e-07, -3.0653e-08, 2.13715e-09,
	 8.79623e-10, 3.40078e-11, 1.18234e-12, 1.33077e-12, 1.26268e-13,
			   0,-5.58943e-07, 3.74589e-09, -6.9056e-09,-2.43765e-10,
	-4.37019e-11, 6.40909e-14,-1.58887e-12, 1.65302e-13,-9.94311e-15,
			   0, 9.55523e-07, -2.4364e-08, 3.27888e-09,-1.98842e-10,
	-3.03638e-11,-1.96969e-12,-3.37035e-13,  2.0621e-14, 2.57239e-15,
	-1.20541e-15,           0,-8.48307e-07, 1.61862e-08, 1.55379e-09,
	  3.9263e-10, 8.45997e-12, 1.02149e-12, 9.72859e-14, 2.05363e-15,
	-4.43586e-16,-1.24253e-16,-3.12276e-17,           0, 5.28855e-07,
	-2.21565e-08, 3.87302e-09, 2.67086e-11, 6.34969e-12, 3.14656e-13,
	 5.92324e-14,-6.62369e-16, 1.99098e-17,-9.04233e-17,-1.03808e-17,
	 2.04133e-18 };
		const static double bnm_bw_A2[91] =
		{ 0,           0, 1.32525e-06,           0, 8.95742e-07,
	 3.54144e-07,           0,-7.41892e-07, 1.20783e-07, 1.63258e-08,
			   0,-3.75263e-07,-9.13671e-08,-3.97264e-08,  7.0578e-09,
			   0,-1.19909e-06,-2.41512e-08, 4.88528e-09,-9.97186e-10,
	 8.72353e-10,           0, 7.10612e-07,-3.56484e-09, -1.0374e-09,
	 3.63659e-10,-6.65795e-12, 5.64715e-11,           0,-3.12199e-07,
	 -9.6864e-09,-2.69482e-09, 3.14226e-10, 5.18676e-11,-4.17524e-11,
	-5.75846e-12,           0, 4.09111e-07,-1.46732e-08, 1.02983e-09,
	-7.44466e-10, 4.26176e-11, 5.18157e-12,-1.02426e-12,-1.23509e-13,
			   0,-5.54183e-07, 3.42429e-08, 3.56586e-09,-9.31467e-11,
	-2.32871e-11, 2.41049e-12,-3.13557e-13,-5.77094e-15,-2.40992e-15,
			   0, 4.02436e-07, -2.5912e-08, 1.37481e-09,-1.46592e-11,
	-5.02333e-12, -5.2097e-12, 1.37394e-14,-3.43718e-14,-3.21568e-17,
	-3.85804e-16,           0,-7.08798e-07, 3.25597e-08,-3.81468e-10,
	-1.43067e-10, 4.71474e-12, 1.63064e-13, 5.42642e-14, 5.38599e-15,
	 5.57437e-16, 6.00857e-17,-2.42002e-17,           0,  4.2816e-07,
	-3.37072e-08, 2.92242e-10,-1.31597e-10, 5.99522e-12,-2.41857e-13,
	 8.06626e-15, 4.16303e-15,-6.99974e-16, 2.20847e-17, 1.15091e-18,
	-1.72364e-19 };
		const static double bnm_bw_B2[91] =
		{ 0,           0, 1.53075e-06,           0, 3.44416e-07,
	-1.68873e-07,           0,-9.23923e-08,-1.38726e-07,-2.57958e-08,
			   0, 3.69384e-07, 6.93156e-09,-1.93088e-09,-1.00996e-08,
			   0,-5.67562e-08, 4.45426e-08, 1.06277e-08,  8.7323e-10,
	-8.04562e-10,           0, 2.26499e-07,-1.56635e-08,  4.5784e-09,
	 4.39321e-10, 4.71733e-11,-1.08949e-12,           0,-1.05598e-07,
	-7.90399e-08, 1.56551e-09,   -3.62e-10, 5.37562e-11,-2.16738e-12,
	 -2.3218e-12,           0,-9.85414e-08,-2.30661e-08,-1.64945e-10,
	-7.17125e-11,  8.2319e-11, -1.5333e-12, 3.10385e-13,-1.81842e-13,
			   0,-3.95581e-08, -2.0084e-08,  7.3331e-11,-3.17491e-10,
	 4.19757e-11,-8.20746e-14, 5.14524e-14, 9.22219e-14,-2.19197e-14,
			   0,-2.80682e-07, 3.35015e-08,-1.10677e-10,  9.3571e-11,
	-8.03417e-12,-1.62657e-12, 4.41398e-14,-1.21694e-14,  5.2966e-15,
	 3.46607e-16,           0,-1.27469e-07, 3.40869e-09, 2.53316e-09,
	 3.74959e-12, 5.41881e-12, 1.74049e-13, 8.18797e-14, -2.6893e-15,
	-3.95923e-16,   3.538e-18,-1.69767e-17,           0,-1.57362e-07,
	 1.47454e-08,-1.86619e-11, 3.14165e-11,-4.46275e-12, 4.47728e-13,
	 2.33252e-14, 7.06514e-15,-2.69505e-17, 7.62909e-17, 4.66794e-19,
	-1.18099e-18 };

		const static double bnm_ch_A0[91] =
		{ 0,           0, 3.44092e-05,           0,-2.60126e-05,
	 8.74012e-06,           0, 3.77497e-05,-7.68623e-07,-2.37093e-06,
			   0,-6.10254e-05,-9.60655e-06,-2.25743e-08,-2.25744e-07,
			   0, 7.45721e-06,-5.77518e-06, 4.52962e-07, 7.20537e-10,
	-2.46254e-08,           0,-6.01147e-05,-2.85938e-06, 7.77123e-07,
	  1.0576e-07, 7.99832e-09, 3.07727e-09,           0,-1.16335e-05,
	-3.95875e-06, 2.29082e-07, 1.44632e-09, 1.77236e-09, 4.85932e-10,
	-3.20174e-11,           0, -1.3601e-05,  2.0002e-06,-8.13287e-09,
	 4.01291e-08,-1.06149e-09, -1.2505e-11, 7.47196e-12,-7.90534e-13,
			   0, 3.12855e-05, 1.56871e-06,-9.99177e-08, 1.24873e-08,
	 1.11519e-10, 1.50889e-10,-1.91145e-12,-2.73249e-12, 8.06637e-13,
			   0, 2.02454e-05,-3.21839e-08,-1.57378e-07, -3.2608e-10,
	-4.58725e-11, 7.60234e-11,-8.92919e-12, 1.91518e-12, 9.09937e-14,
	-4.63611e-14,           0, 6.17105e-06,-2.44014e-07, 8.18199e-08,
	 7.66269e-11,-1.84076e-10,-2.93307e-11,-9.59054e-13,  2.8017e-13,
	-3.04524e-14, 5.98765e-15,-2.37069e-15,           0,-2.71386e-05,
	-3.93656e-08, 5.25319e-08, 1.87207e-09,-1.08295e-10,-4.95118e-12,
	 6.68583e-13, 5.80963e-14,-4.10568e-15,  1.8467e-17, 2.65355e-16,
	 9.94267e-17 };
		const static double bnm_ch_A1[91] =
		{ 0,           0,-1.21877e-05,           0,  1.7057e-05,
	-2.25874e-06,           0,-1.08199e-05,-4.01401e-06,-9.52317e-07,
			   0,-2.51816e-05,-7.31382e-06, 2.33059e-07, -1.4443e-07,
			   0,-3.81397e-05,-3.93061e-06,  9.8099e-07, 2.75612e-09,
	-3.10301e-09,           0,-3.16632e-05,-3.67776e-06, 3.60896e-07,
	  2.5801e-08, 1.67424e-09, 3.93584e-10,           0,-1.39865e-05,
	-1.17976e-07, 5.72962e-07, 6.19368e-09,-8.14389e-10, 2.24864e-10,
	-1.29872e-11,           0, 1.77873e-05, 2.43393e-06, 1.05207e-08,
	 2.92634e-08,-4.39748e-10, 2.05203e-10, -2.5137e-11, 3.55825e-12,
			   0, 1.31629e-05, 1.89227e-06,-1.10785e-07, 1.10663e-08,
	-4.21966e-10,  1.0313e-10,-1.11356e-11,-1.65089e-13, 1.30893e-13,
			   0, -9.7772e-06,  8.3876e-07,-7.86804e-08,-3.14577e-09,
	-6.87848e-11, 4.76464e-11,-4.38525e-12, 3.06846e-13, 7.20651e-15,
	 1.73908e-15,           0,-1.04343e-05, 2.06992e-07,-3.20013e-08,
	-6.07862e-10,-1.30985e-10, 2.88627e-11, 6.04803e-13, 9.09573e-14,
	 3.42846e-14,-2.63942e-15, 2.87429e-16,           0,-1.41835e-05,
	-6.36433e-07, 9.00866e-09,-1.46428e-09, 4.75553e-11, 5.31084e-13,
	 7.38289e-13, 6.93474e-14, 1.62253e-14, 8.28473e-17,-3.70233e-17,
	 4.17029e-18 };
		const static double bnm_ch_B1[91] =
		{ 0,           0,-1.87491e-05,           0, 3.08332e-05,
	 6.50985e-07,           0,-1.67718e-05, -2.1639e-06,-2.22722e-07,
			   0, 2.01046e-05,-2.52768e-06, 2.24747e-07, -2.9681e-08,
			   0,-1.41086e-05, -2.1737e-06, 4.67492e-07, 2.47772e-09,
	 8.06559e-09,           0, 4.10038e-06,-5.06446e-07,  3.8861e-07,
	-1.51356e-08, 2.94737e-09, 3.86722e-11,           0, 2.52546e-06,
	  7.0319e-07,  4.2197e-08,  1.6711e-09, 4.50422e-09,-2.22535e-10,
	-4.24093e-11,           0, 4.80989e-06, 1.21479e-06,  6.5404e-09,
	  6.0418e-09,  8.8304e-10, 1.37818e-11,-2.12197e-11, 8.00836e-13,
			   0, 2.65598e-06,  4.1562e-07,-1.10318e-07, 5.40659e-09,
	 3.26786e-10,-3.09482e-11,-1.76207e-11,-2.20125e-13, 9.72079e-14,
			   0,-4.35468e-06,-5.08059e-07,-7.40445e-08, 8.46796e-10,
	 8.18226e-12,-2.47198e-11,-4.02709e-12, -2.4483e-13, 2.45384e-14,
	-4.41651e-15,           0,-1.72712e-05,-3.85806e-07, 5.94001e-08,
	 4.95796e-10,-1.52547e-10, 5.58812e-12,-9.80016e-13, 2.16449e-14,
	 2.44231e-14,-1.80205e-15,-1.67056e-16,           0,-2.00778e-07,
	-2.47887e-07, 5.00693e-08,-2.71812e-10,-5.49429e-11, -1.9368e-12,
	  5.4763e-13, 6.60728e-15, 1.00775e-14,-1.53401e-16, 3.52689e-18,
	-7.65153e-21 };
		const static double bnm_ch_A2[91] =
		{ 0,           0, -2.6098e-05,           0, 1.66257e-05,
	 1.30425e-06,           0,-3.22476e-05,-1.76913e-06,-6.25158e-08,
			   0, 7.21108e-06,  9.0904e-07, 6.78551e-08,-5.93859e-08,
			   0,-2.28515e-05,-1.48061e-07,-8.31032e-09, 4.30622e-09,
	 2.98197e-10,           0, 3.55215e-07, 8.21777e-07,-4.39534e-08,
	-1.13336e-09,-1.56727e-09, 1.72181e-10,           0,-8.79153e-06,
	 3.38907e-07, 1.24113e-08, 2.57979e-09, 5.67452e-10,-7.96396e-11,
	  2.6657e-11,           0,  3.4686e-06, 1.95583e-07,-1.96403e-08,
	 8.61849e-10, 3.49392e-10, 6.82058e-11, 1.55282e-11,  8.7346e-13,
			   0, 8.68923e-06,-2.74254e-07, 6.22727e-09,-2.79119e-09,
	 1.93488e-10,-4.47657e-11,  8.1529e-12, 5.32589e-13,  3.8741e-14,
			   0,   2.196e-06, 4.16177e-08,  3.1526e-08,-2.59329e-09,
	 1.81082e-10, -8.8344e-12, 4.84344e-12, 7.86297e-14,-4.79581e-15,
	-6.61409e-16,           0,-8.16816e-07, 1.67162e-08, 2.24122e-08,
	-3.07589e-10,   -2.51e-11,-1.68693e-13,  1.7053e-12,  1.1555e-13,
	 5.76018e-16,-1.84982e-16, 2.72111e-18,           0,  5.9433e-07,
	-2.64906e-08, 2.47269e-08, 8.44902e-10,-6.60908e-11,-1.61771e-12,
	 -1.0077e-13,-5.21029e-15,-2.44793e-16,-5.01061e-17,-8.62216e-18,
	-5.62131e-18 };
		const static double bnm_ch_B2[91] =
		{ 0,           0, 4.31639e-06,           0,-1.07841e-05,
	-1.85081e-07,           0, 1.12282e-05, -1.1274e-06, 1.86582e-08,
			   0,-1.30692e-05,-6.76455e-07, 1.25076e-07, -2.4321e-08,
			   0, 7.28639e-06,  -2.742e-07, 1.69426e-07,-2.86498e-08,
	 6.32504e-09,           0,-2.23607e-06, -5.9869e-07,-6.26882e-08,
	 5.37471e-10, 8.46187e-10,-2.16916e-10,           0,-8.97665e-06,
	-3.67714e-07, 9.56404e-08,-6.90807e-09,  2.4761e-09, 3.12587e-12,
	-5.25165e-12,           0,-1.73248e-06,-3.11848e-07, -1.4038e-08,
	 5.98065e-09, 8.57722e-10,-9.41516e-11, 9.99224e-12, 1.34176e-12,
			   0,-7.51164e-06,-4.28826e-07,-3.39215e-08,  -2.475e-09,
	 7.00775e-10,-7.36245e-12, 3.45079e-12, 5.70009e-13, -5.5641e-14,
			   0,-3.26671e-06, 1.53843e-07, 5.60536e-09,-8.01055e-10,
	 1.74511e-10, 5.93967e-13, 5.12869e-12, 2.72348e-13, 3.64605e-14,
	-1.60016e-15,           0,  3.4279e-06,  4.1514e-07,-1.33796e-08,
	 3.44456e-10,-1.93924e-11, 1.80464e-12, 1.70459e-12, 4.97773e-14,
	-9.74409e-15,-5.85073e-16, 8.46647e-17,           0, 8.61857e-06,
	  1.1069e-07,-7.27649e-09,-5.62684e-10,-5.97347e-11, 1.23277e-11,
	-1.65565e-13,-1.11284e-16,-1.59284e-15,-2.20728e-16, 9.26909e-18,
	-3.03733e-18 };

		const static double bnm_cw_A0[91] =
		{ 0,           0,-0.000209105,           0,-0.000186337,
	 0.000108195,           0, 0.000119941,-1.77495e-05,-1.47564e-05,
			   0,   4.933e-05, -2.8045e-05, 1.88835e-06,-4.69919e-06,
			   0, 1.77051e-05, 9.25606e-06, 4.28234e-06, 8.50394e-07,
	-2.87064e-07,           0,-1.95692e-05, 5.34497e-06, 3.39951e-07,
	 2.15793e-07, 2.11953e-09,-2.08402e-09,           0, 0.000124901,
	-8.38934e-06, 3.72965e-07,-2.32149e-07,-2.92636e-09,-1.26453e-09,
	 9.92539e-10,           0,-0.000182885,-5.21449e-06, 2.37091e-07,
	 7.24371e-08,-1.22127e-08,-5.44137e-10, 1.08394e-10,-3.41447e-11,
			   0, 2.91161e-05, 1.24267e-06,-1.70671e-08, 1.14115e-08,
	-1.19144e-09, 4.94619e-10,  1.3396e-10, 1.30277e-11,-4.40721e-12,
			   0,-4.17667e-05, 9.48491e-07,-2.67546e-07,-5.15114e-08,
	 2.04636e-11,-2.41897e-11, 7.55171e-12, 2.41883e-12, 4.16996e-13,
	 3.56938e-14,           0, 0.000117579, -1.4327e-06, 3.06089e-07,
	-9.94255e-09,-9.20752e-10,-1.43544e-11,-7.88054e-12,-1.21559e-12,
	-4.15628e-13,-3.91838e-14, 9.25912e-15,           0,-0.000103762,
	-1.18428e-06, 1.51836e-07,-3.77758e-09,-1.07948e-09,-2.00386e-12,
	 2.73094e-12, 6.53946e-13,-1.09122e-13,-1.41883e-14,-1.17256e-15,
	 2.66203e-16 };
		const static double bnm_cw_A1[91] =
		{ 0,           0, -1.4153e-05,           0, 0.000191257,
	-6.97051e-05,           0,-7.70548e-05,-1.37061e-05, 2.08891e-06,
			   0,-6.77641e-05,-1.04368e-05, 5.69126e-07,-7.00779e-07,
			   0,-3.18353e-05,-8.25179e-06, 2.10471e-07, 1.58765e-07,
	 4.50394e-08,           0, 5.15486e-05, 3.90973e-06,-5.63444e-07,
	 2.05911e-07, 4.04318e-08,-8.50244e-09,           0, -5.7077e-05,
	  1.5686e-06, 6.98175e-07,-4.21889e-08, 3.38086e-09,  1.9131e-09,
	-2.96318e-10,           0, 7.27425e-05,-6.70572e-06, 3.58428e-07,
	-7.43945e-09,-3.89759e-09,-4.81035e-10,-7.99525e-11, 2.72676e-11,
			   0,-7.24641e-05,-4.13127e-06, 1.36891e-08, 2.79308e-08,
	-1.28198e-09,-1.01554e-10,-1.87141e-11,-4.95313e-12, 2.99575e-12,
			   0, 1.39005e-05,  1.6775e-06,-3.31848e-08, 8.83631e-09,
	-1.07924e-09, 1.33548e-10,-1.57123e-11, 2.97347e-12,-1.71585e-13,
	 2.43013e-14,           0,-4.52648e-05, 1.21565e-06, 9.79084e-08,
	 1.43251e-08,-9.44313e-10, 1.78464e-10,-3.20208e-12, 5.31301e-13,
	 2.77621e-13, 9.74715e-15,-4.02622e-15,           0, 4.38145e-05,
	-1.30189e-06, 1.11363e-07, 1.23324e-08, 5.26052e-10, 2.54704e-11,
	 5.08901e-12,-2.24958e-13, 8.36362e-14, 1.00455e-14, 1.05649e-16,
	-7.67225e-17 };
		const static double bnm_cw_B1[91] =
		{ 0,           0, 3.00319e-05,           0, 7.28356e-05,
	-6.68037e-05,           0,-8.15376e-05,-1.74806e-05, 3.19877e-06,
			   0,-3.25043e-05, 1.64474e-07,-2.22757e-06, 2.97544e-09,
			   0, 3.04232e-05, 6.74033e-06, -4.7505e-07,-2.16268e-07,
	 6.63315e-08,           0, 3.00853e-05, 3.70149e-06, 4.52082e-07,
	 1.33748e-08, 2.40972e-09, 2.60895e-09,           0,-8.44887e-05,
	-1.77705e-06, 1.75761e-08, 8.35926e-08, 4.72187e-09, 1.54756e-09,
	 1.17466e-10,           0, 3.05286e-05,-3.95473e-06, 2.55709e-07,
	 8.61752e-10,  4.1289e-11,-2.56102e-11,  1.7385e-10, 5.69675e-12,
			   0,-8.58324e-06,-8.47496e-07,-2.80902e-07,-1.71145e-08,
	-1.22314e-09,-3.58809e-10,-3.04265e-11,  2.2307e-12,-1.54917e-12,
			   0, 1.46521e-05, 6.80159e-07, 1.05837e-07, 3.36579e-09,
	-6.96482e-10,-5.13535e-11,-1.48874e-11,  1.2872e-12,-7.23064e-14,
	-7.96091e-14,           0, -2.6913e-05, 8.53701e-07, 7.96525e-08,
	 1.93912e-08, 7.24197e-11,  7.6995e-11, 1.01527e-12, 3.44024e-13,
	-8.99784e-14, 6.79982e-15,-2.43952e-15,           0, 2.43407e-05,
	-1.88014e-06, 1.55375e-07,  1.7251e-09,-3.67658e-11, 4.08564e-12,
	-7.66669e-13, 6.74639e-15, -3.9075e-14, 2.63347e-15, 1.31778e-16,
	 3.51487e-18 };
		const static double bnm_cw_A2[91] =
		{ 0,           0,-1.82864e-05,           0, 3.59638e-05,
	 2.30388e-05,           0, 1.06006e-05,-6.91746e-07, 9.43985e-07,
			   0, 8.33227e-06,-3.57421e-06,-1.96699e-07, 3.86101e-07,
			   0,-6.26821e-05, 3.22192e-06,-4.89383e-07,-7.43341e-10,
	 7.61199e-08,           0, 1.21485e-05, 5.73731e-08,-2.57095e-07,
	-2.07998e-08, 8.56289e-09,-6.69157e-10,           0,-3.11443e-05,
	-5.70219e-08, 1.59731e-07,-2.24572e-08,-8.32354e-11,-1.09865e-09,
	-5.00186e-10,           0, 2.55324e-05,-6.41023e-07, 8.44594e-08,
	-2.34651e-08, 2.06528e-09, 3.21881e-10, -8.0639e-11,-3.38403e-12,
			   0,-1.14037e-05, 5.48723e-07, -5.4537e-09, 4.86116e-09,
	 6.23116e-10,   1.235e-10,-1.42908e-11,  1.6645e-12, 8.90016e-14,
			   0, 3.23485e-05,-1.34558e-06, 1.55588e-07, -6.2235e-09,
	-4.70239e-10,-8.32767e-11,-7.10529e-13,-2.49241e-12, 2.45406e-13,
	-1.59549e-14,           0,-3.82266e-05,  1.5271e-06, 4.54771e-08,
	-6.52215e-09,-6.71801e-11, -4.2239e-12, 6.02788e-12,-6.22598e-14,
	 1.07254e-13,-2.41421e-15,-1.97007e-15,           0, 7.89104e-06,
	-1.01342e-06, 1.94451e-09, -1.8861e-09,  3.1611e-10, -4.8335e-11,
	 2.50016e-12, 3.28538e-14,-5.44916e-14, 1.53043e-15, 1.44815e-16,
	 4.10287e-17 };
		const static double bnm_cw_B2[91] =
		{ 0,           0,-7.62965e-06,           0,-2.53927e-05,
	-1.22735e-05,           0, 2.31177e-05,-7.10232e-06,-4.90481e-06,
			   0,   8.115e-06, 2.95887e-06,-2.91861e-07,  2.3094e-07,
			   0,-1.75095e-06, 6.09415e-06, 8.75232e-07, 1.75132e-07,
	-6.05694e-09,           0,-6.72451e-06, 5.52258e-07,-6.84886e-08,
	-3.69813e-08, 2.31035e-08, -5.1628e-09,           0,-1.12983e-05,
	-4.30378e-06, 3.62364e-07,-6.92114e-08, 4.19674e-09, 1.83645e-10,
	-8.54778e-11,           0, -6.3986e-06,-3.11616e-06, 9.27243e-09,
	 2.94053e-09, 1.73489e-09,-2.70195e-11,-7.63143e-13,-2.96732e-12,
			   0, 1.32245e-05,-1.98289e-06,-9.58796e-08,-8.13061e-09,
	 2.11528e-09,-1.21018e-10,-1.09874e-11, 6.26223e-12,-1.59135e-12,
			   0,-8.57936e-06, 1.62108e-06,-2.84996e-08, 5.03959e-09,
	-6.60278e-10, 5.72615e-11,-7.99006e-12, 6.71156e-13, 4.43533e-13,
	 8.99104e-15,           0,-4.36549e-06, 1.22383e-06, 2.22842e-07,
	-1.97114e-09, 2.33147e-10, 3.05176e-11,  1.1492e-11,-5.47032e-14,
	  6.8599e-14,-2.20889e-15, 1.03066e-16,           0,-1.66841e-05,
	 9.21813e-07,-1.73094e-08, 1.28938e-09,-3.24273e-10,-3.98153e-13,
	 4.29763e-12,   2.542e-13, 2.43817e-14, 2.49081e-15,-3.72533e-16,
	-6.72172e-17 };
		// date
		int iyear = 0;
		int idoy = 0;;
		mjd2doy((int)(mjd), &iyear, &idoy);
		double doy = mjd - (int)(mjd)+idoy;

		// conversions
		double el = G_PI / 2 - zd;
		double polDist = G_PI / 2 - lat;

		//a.) calculate Legendre polynomials
		//degree nand order m
		int nmax = 12;

		//unit vector
		double x = sin(polDist) * cos(lon);
		double y = sin(polDist) * sin(lon);
		double z = cos(polDist);

		//Legendre polynomials
		double V[13][13] = { 0 };
		double W[13][13] = { 0 };
		V[0][0] = 1;
		W[0][0] = 0;
		V[1][0] = z * V[0][0];
		W[1][0] = 0;

		for (int i = 1; i < nmax; i++)
		{
			int num_i = i + 1; 
			V[i + 1][0] = ((2 * num_i - 1) * z * V[i][0] - (num_i - 1) * V[i - 1][0]) / num_i;
			W[i + 1][0] = 0;
			//printf("%d\t%.6f\n", i, V[i + 1][0]);
		}

		for (int m = 0; m < nmax; m++)
		{
			int num_m = m + 1;
			V[m + 1][m + 1] = (2 * num_m - 1) * (x * V[m][m] - y * W[m][m]);
			W[m + 1][m + 1] = (2 * num_m - 1) * (x * W[m][m] + y * V[m][m]);
			if (m < nmax - 1)
			{
				V[m + 2][m + 1] = (2 * num_m + 1) * z * V[m + 1][m + 1];
				W[m + 2][m + 1] = (2 * num_m + 1) * z * W[m + 1][m + 1];
			}
			for (int n = m + 2; n < nmax; n++)
			{
				int num_n = n + 1;
				V[n + 1][m + 1] = ((2 * num_n - 1) * z * V[n][m + 1] - (num_n + num_m - 1) * V[n - 1][m + 1]) / (num_n - num_m);
				W[n + 1][m + 1] = ((2 * num_n - 1) * z * W[n][m + 1] - (num_n + num_m - 1) * W[n - 1][m + 1]) / (num_n - num_m);
			}
		}

		//b.) determine the coefficients bh, bw, chand cw
		//initialize
		double bh_A0 = 0.0;
		double bh_A1 = 0.0;
		double bh_B1 = 0.0;
		double bh_A2 = 0.0;
		double bh_B2 = 0.0;
		double bw_A0 = 0.0;
		double bw_A1 = 0.0;
		double bw_B1 = 0.0;
		double bw_A2 = 0.0;
		double bw_B2 = 0.0;
		double ch_A0 = 0.0;
		double ch_A1 = 0.0;
		double ch_B1 = 0.0;
		double ch_A2 = 0.0;
		double ch_B2 = 0.0;
		double cw_A0 = 0.0;
		double cw_A1 = 0.0;
		double cw_B1 = 0.0;
		double cw_A2 = 0.0;
		double cw_B2 = 0.0;

		int i = 0;
		for (size_t n = 0; n < nmax + 1; n++)
		{
			for (size_t m = 0; m < n + 1; m++)
			{
				int num_i = i + 1;

				bh_A0 += anm_bh_A0[i] * V[n][m] + bnm_bh_A0[i] * W[n][m];
				//printf("anm_bh_A0[i] %.8f\n", anm_bh_A0[i]);
				//printf("V[n][m] %.8f\n", V[n][m]);
				//printf("bnm_bh_A0[i] %.8f\n", bnm_bh_A0[i]);
				//printf("W[n][m] %.8f\n", W[n][m]);

				bh_A1 += anm_bh_A1[i] * V[n][m] + bnm_bh_A1[i] * W[n][m];
				bh_B1 += anm_bh_B1[i] * V[n][m] + bnm_bh_B1[i] * W[n][m];
				bh_A2 += anm_bh_A2[i] * V[n][m] + bnm_bh_A2[i] * W[n][m];
				bh_B2 += anm_bh_B2[i] * V[n][m] + bnm_bh_B2[i] * W[n][m];

				bw_A0 += anm_bw_A0[i] * V[n][m] + bnm_bw_A0[i] * W[n][m];
				bw_A1 += anm_bw_A1[i] * V[n][m] + bnm_bw_A1[i] * W[n][m];
				bw_B1 += anm_bw_B1[i] * V[n][m] + bnm_bw_B1[i] * W[n][m];
				bw_A2 += anm_bw_A2[i] * V[n][m] + bnm_bw_A2[i] * W[n][m];
				bw_B2 += anm_bw_B2[i] * V[n][m] + bnm_bw_B2[i] * W[n][m];

				ch_A0 += anm_ch_A0[i] * V[n][m] + bnm_ch_A0[i] * W[n][m];
				ch_A1 += anm_ch_A1[i] * V[n][m] + bnm_ch_A1[i] * W[n][m];
				ch_B1 += anm_ch_B1[i] * V[n][m] + bnm_ch_B1[i] * W[n][m];
				ch_A2 += anm_ch_A2[i] * V[n][m] + bnm_ch_A2[i] * W[n][m];
				ch_B2 += anm_ch_B2[i] * V[n][m] + bnm_ch_B2[i] * W[n][m];

				cw_A0 += anm_cw_A0[i] * V[n][m] + bnm_cw_A0[i] * W[n][m];
				cw_A1 += anm_cw_A1[i] * V[n][m] + bnm_cw_A1[i] * W[n][m];
				cw_B1 += anm_cw_B1[i] * V[n][m] + bnm_cw_B1[i] * W[n][m];
				cw_A2 += anm_cw_A2[i] * V[n][m] + bnm_cw_A2[i] * W[n][m];
				cw_B2 += anm_cw_B2[i] * V[n][m] + bnm_cw_B2[i] * W[n][m];

				i += 1;
			}
		}

		// adding the seasonal amplitudes for the specified doy to the mean values
		double bh = bh_A0 + bh_A1 * cos(doy / 365.25 * 2 * G_PI)
			+ bh_B1 * sin(doy / 365.25 * 2 * G_PI)
			+ bh_A2 * cos(doy / 365.25 * 4 * G_PI)
			+ bh_B2 * sin(doy / 365.25 * 4 * G_PI);
		double bw = bw_A0 + bw_A1 * cos(doy / 365.25 * 2 * G_PI)
			+ bw_B1 * sin(doy / 365.25 * 2 * G_PI)
			+ bw_A2 * cos(doy / 365.25 * 4 * G_PI)
			+ bw_B2 * sin(doy / 365.25 * 4 * G_PI);
		double ch = ch_A0 + ch_A1 * cos(doy / 365.25 * 2 * G_PI)
			+ ch_B1 * sin(doy / 365.25 * 2 * G_PI)
			+ ch_A2 * cos(doy / 365.25 * 4 * G_PI)
			+ ch_B2 * sin(doy / 365.25 * 4 * G_PI);
		double cw = cw_A0 + cw_A1 * cos(doy / 365.25 * 2 * G_PI)
			+ cw_B1 * sin(doy / 365.25 * 2 * G_PI)
			+ cw_A2 * cos(doy / 365.25 * 4 * G_PI)
			+ cw_B2 * sin(doy / 365.25 * 4 * G_PI);

		//calculating the hydrostaticand wet mapping factors
		double mfh = (1 + (ah / (1 + bh / (1 + ch)))) / (sin(el) + (ah / (sin(el) + bh / (sin(el) + ch))));
		double mfw = (1 + (aw / (1 + bw / (1 + cw)))) / (sin(el) + (aw / (sin(el) + bw / (sin(el) + cw))));

		//height correction for the hydrostatic part[Niell, 1996]
		double a_ht = 2.53e-05;
		double b_ht = 5.49e-03;
		double c_ht = 1.14e-03;
		double h_ell_km = h_ell / 1000;//convert height to km
		double ht_corr_coef = 1 / sin(el) - (1 + (a_ht / (1 + b_ht / (1 + c_ht)))) / (sin(el) + (a_ht / (sin(el) + b_ht / (sin(el) + c_ht))));
		double ht_corr = ht_corr_coef * h_ell_km;
		mfh += ht_corr;

		*dmap = mfh;
		*wmap = mfw;

		return;
	}

	double t_gpt3::asknewet_gpt3(double e, double Tm, double lambda)
	{
		//coefficients
		double k1 = 77.604;                       // K/hPa
		double k2 = 64.79;                        // K/hPa
		double k2p = k2 - k1 * 18.0152 / 28.9644; // K/hPa
		double k3 = 377600;                       // KK/hPa

		// mean gravity in m / s * *2
		double gm = 9.80665;
		// molar mass of dry air in kg / mol
		double dMtr = 28.965 * 1e-3;
		// universal gas constant in J / K / mol
		double R = 8.3143;

		// specific gas constant for dry consituents
		double Rd1 = R / dMtr;

		double zwd = 1e-6 * (k2p + k3 / Tm) * Rd1 / (lambda + 1) / gm * e;
		return zwd;
	}

	bool t_gpt3::has_gpt3_data()
	{
		return (fabs(_p_gpt3) > 1e-9);
	}


} // namespace
