/**
 * @file mem_diag.h
 * @brief Dual-core memory and stack diagnostic polling helpers.
 */

 #ifndef DCO_MEM_DIAG_H
 #define DCO_MEM_DIAG_H
 
 #ifdef ENABLE_MEM_DIAG
 
 #include <stdbool.h>
 
 extern volatile bool mem_diag_runtime_enabled;
 extern volatile bool mem_diag_pending;
 extern volatile bool mem_diag_core1_ready;
 
 void mem_diag_request();
 void mem_diag_poll_core0_work();
 void mem_diag_poll_core1_work();
 
 /**
  * @brief Core 1 memory diagnostic poll hook called during loop1().
  */
 static inline void mem_diag_poll_core1() {
   if (!mem_diag_runtime_enabled) return;
   if (!mem_diag_pending || mem_diag_core1_ready) return;
   mem_diag_poll_core1_work();
 }
 
 /**
  * @brief Core 0 memory diagnostic poll hook called during loop().
  */
 static inline void mem_diag_poll_core0() {
   if (!mem_diag_runtime_enabled) return;
   if (!mem_diag_pending || !mem_diag_core1_ready) return;
   mem_diag_poll_core0_work();
 }
 
 #else  // !ENABLE_MEM_DIAG
 
 static inline void mem_diag_request() {}
 static inline void mem_diag_poll_core0() {}
 static inline void mem_diag_poll_core1() {}
 
 #endif // ENABLE_MEM_DIAG
 
 #endif // DCO_MEM_DIAG_H