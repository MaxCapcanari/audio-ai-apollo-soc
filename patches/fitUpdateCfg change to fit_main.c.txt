I manually made a change to fit_main.c in order to significantly improve BLE audio download performance. 
It let me reduce OPUS_INTER_PKT_DELAY_MS from 40 down to 10. In the end, I kept it at 15. at both 15 and 10 the download speed of a 10s OPUS file was about 2.8 seconds. 
It was a one-statement change (see end of this file). 
fit.main.c is not in the git repository. You find it in /mnt/c/AmbiqSuite-R4-5-0/AmbiqSuite_R4.5.0/AmbiqSuite_R4.5.0/third_party/cordio/ble-profiles/sources/apps/fit/fit_main.c
I implemented it after the main patch that the student wrote.

Stock values:          Change to:
  48,  /* min iv */      12,  /* min iv (15 ms) */
  60,  /* max iv */      24,  /* max iv (30 ms) */
  4,   /* latency */     0,   /* latency */
  
the stock 48-60 / latency-4 profile means the peripheral can only transmit every 60-75 ms and may skip up to four events. AttsHandleValueNtf() returns void, so notifications sent faster are dropped silently. With stock values OPUS_INTER_PKT_DELAY_MS must be 40 or higher (8.7 s per 10-second recording); with the change it can be 15 (2.8 s).

Note how to check it's applied. Watch onConnectionUpdated in the Flutter debug log — you want interval 24 and latency 0. If it shows 60 and 4, the SDK edit is missing.


static const appUpdateCfg_t fitUpdateCfg =
{
  100,                                    /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  12,                                     /*! Minimum connection interval in 1.25ms units (15 ms) */
  24,                                     /*! Maximum connection interval in 1.25ms units (30 ms) */
  0,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};