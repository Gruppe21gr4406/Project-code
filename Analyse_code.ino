// Variabler til find_thr_ECG()

int decrement = -1;
bool stop_thr_R = 0;
int search_thr = 4095;

//R variabler
float percent_R = 0.10;
int cnt_R = 0;
int duration_R = 60;
int time_since_last_R = duration_R + 1;
int holder_R_peak = 0;
float comparison_R_peak = 0;

//T variabler
int search_time_T = 400;
int holder_T_wave = 0;
float comparison_T_wave = 0;
int cnt_array = 0;
int cnt_T = 0;

// SKG
int search_SCG_duration = 150;
uint32_t comparison_AC_max = 0;
int holder_AC_max = 0;

int search_time_AC_min = 10;
uint32_t comparison_AC_min = 0;
int holder_AC_min = 0;
int32_t AC_test = 0;

// find_thr_ECG starter fra den højeste værdi vi KAN sample, og decrementes, indtil den finder den højeste værdi der ER samplet
void find_thr_ECG()
{
  for (float g = search_thr ; g > 0 ; g += decrement) {
    for (int i = 0; i < sizeof(ekg_SPIFFS) / sizeof(int32_t) ; i++) {
      if (g <= ekg_SPIFFS[i]) {
        locate_thr_R = g;
        stop_thr_R = 1;
      }
    }
    if (stop_thr_R == 1) {
      break;
    }
  }
}

// detect_events() finder R takker, T takker, AC-max samt AC-min.
void detect_events ()
{
  //Detekterer R-takker
  int thr_R = locate_thr_R * (1 - percent_R) ;
  for (int t = 0; t < sizeof(ekg_SPIFFS) / sizeof(int32_t) ; t++) {
    if (cnt_R > 0) {
      time_since_last_R = t - holder_R_peak;
    }
    if (ekg_SPIFFS[t] > thr_R && duration_R < time_since_last_R && t < sizeof(ekg_SPIFFS) / sizeof(int32_t) - search_time_T - search_SCG_duration) {
      cnt_R = cnt_R + 1;
      comparison_R_peak = ekg_SPIFFS[t];
      int start_search = t;
      for (int i = start_search; i < start_search + duration_R; i++) {
        if (comparison_R_peak <= ekg_SPIFFS[i]) {
          comparison_R_peak = ekg_SPIFFS[i];
          holder_R_peak = i;
        }
      }
      //Detekterer T-takker
      comparison_T_wave = 0;
      for (int k = start_search + duration_R; k < start_search + search_time_T; k++) {
        if ( comparison_T_wave <= ekg_SPIFFS[k] && k < sizeof(ekg_SPIFFS) / sizeof(int32_t)) {
          comparison_T_wave = ekg_SPIFFS[k];
          holder_T_wave = k;
        }
      }
      //Detekter AC max
      int start_search_SCG = holder_T_wave;        // Vi starter fra værdien af den fundne t-tak og søger efter ACmax.
      comparison_AC_max = skg_SPIFFS[start_search_SCG];
      for (int k = start_search_SCG; k < start_search_SCG + search_SCG_duration; k++) {
        if (comparison_AC_max < skg_SPIFFS[k]) {
          comparison_AC_max = skg_SPIFFS[k];
          holder_AC_max = k;
        }
        //Detekter AC min
        comparison_AC_min = comparison_AC_max;
        int start_reverse_search = holder_AC_max;
        for (int t = start_reverse_search; t > start_reverse_search - search_time_AC_min; t += decrement ) {
          if (comparison_AC_min > skg_SPIFFS[t]) {
            comparison_AC_min = skg_SPIFFS[t];
            holder_AC_min = t;
          }
        }
      }
      AC_test = skg_SPIFFS[holder_AC_max] - skg_SPIFFS[holder_AC_min];
      cnt_R_test++;
      if (AC_test > 2000) {

        AC_amplitude += AC_test;
        cnt_AC++;
        Serial.printf("R-tak: %d \n", holder_R_peak);
        Serial.printf("AC-amplitude: %d \n", skg_SPIFFS[holder_AC_max] - skg_SPIFFS[holder_AC_min]);
      }
    }
  }
  cnt_R = 0;
}

//Samler alle funktionerne i én funktion.
void analyze_ECG_and_SCG() {
  find_thr_ECG();
  detect_events ();
}
