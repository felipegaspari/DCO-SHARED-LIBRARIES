/**
 * @file cv_out.h
 * @brief Analog control voltage (CV) math and modulation depth scaling bake routines.
 */

 #ifndef DCO_CV_OUT_H
 #define DCO_CV_OUT_H
 
 #include <stdint.h>
 
 #ifndef NUM_FILTERS
 #define NUM_FILTERS 2
 #endif
 
 void init_cv_out();
 void update_CV_outs();
 void update_CV_outs_manual_calibration();
 
 void __not_in_flash_func(cv_bake_adsr2_to_vcf_scale)();
 void __not_in_flash_func(cv_bake_lfo2_to_vcf_scale)();
 void __not_in_flash_func(cv_bake_lfo1_to_vca_scale)();
 void cv_update_mod_scales();
 
 #endif // DCO_CV_OUT_H