## CONFIG_ENABLE_BELUGA_UWB
> _Enables DecaDriver library for UWB_
> 
> > **Type:** `bool`
> >
> > **Defaults:** `y`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Beluga UWB Settings`

## CONFIG_UWB_BOOT_BANNER
> _UWB Boot Banner_
> 
> > **Type:** `bool`
> >
> > **Depends:** [CONFIG_ENABLE_BELUGA_UWB](uwb.md#config_enable_beluga_uwb)
> >
> > **Defaults:** `n`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Beluga UWB Settings > Enables DecaDriver library for UWB`

## CONFIG_UWB_INIT_RX_TIMEOUT
> _Initiator receive timeout (ms)_
> 
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_BELUGA_UWB](uwb.md#config_enable_beluga_uwb)
> >
> > **Defaults:** `2700`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Beluga UWB Settings > Enables DecaDriver library for UWB`

## CONFIG_UWB_RESP_RX_DELAY
> _Responder receive delay (ms)_
> 
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_BELUGA_UWB](uwb.md#config_enable_beluga_uwb)
> >
> > **Defaults:** `0`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Beluga UWB Settings > Enables DecaDriver library for UWB`

## CONFIG_UWB_RESP_RX_TIMEOUT
> _Filter UWB neighbors by range_
> 
> Exclude neighbors that are within a certain distance and beyond a certain distance
> 
> > **Type:** `bool`
> >
> > **Depends:** [CONFIG_ENABLE_BELUGA_UWB](uwb.md#config_enable_beluga_uwb)
> >
> > **Defaults:** `n`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Beluga UWB Settings > Enables DecaDriver library for UWB`

## CONFIG_UWB_RANGE_FILTER_LOWER_BOUND
> _Lower bound of the range filtering_
> 
> This is the lower bound of the range filtering. If a lower bound is not desired, then set it to a negative value.
> 
> > **Type:** `int`
> >
> > **Defaults:** `-5`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Beluga UWB Settings`

## CONFIG_UWB_RANGE_FILTER_UPPER_BOUND
> _Upper bound of the range filtering_
> 
> This is the upper bound of the range filtering. If an upper bound is not desired, then set it to a negative value.
> 
> > **Type:** `int`
> >
> > **Defaults:** `-5`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Beluga UWB Settings`

## CONFIG_POLL_RX_TO_RESP_TX_DLY
> _Poll RX to Response Delay in microseconds_
> 
> Critical value for porting to different processors. For slower platforms 
> where the SPI is at a slower speed or the processor is operating at a lower 
> frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz) 
> this value needs to be increased. Knowing the exact time when the responder 
> is going to send its response is vital for time of flight calculation. The specification 
> of the time of response must allow the processor enough time to do its calculations and 
> put the packet in the Tx buffer. So more time required for a slower system (processor).
> 
> > **Type:** `int`
> >
> > **Defaults:** `1500`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_DWT_API_ERROR_CHECK
> _Have DecaDriver check for errors_
> 
> > **Type:** `bool`
> >
> > **Defaults:** `n`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_UWB_ENABLE_PA
> _Allow the UWB external power amplifier to be toggled on_
> 
> > **Type:** `bool`
> >
> > **Defaults:** `y`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_MAX_POLLING_RATE
> _Maximum Polling Rate (ms)_
> 
> > **Type:** `int`
> >
> > **Defaults:** `500`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_POLLING_REFRESH
> _Refresh Period while Polling_
> 
> This determines the maximum amount of ms the ranging task can sleep for; However, it is important to note that this is different than the polling rate. The polling rate is what determines how often the system should poll the distances for other nodes while this parameter indicates the maximum amount of time the polling thread can sleep for, thus allowing for the ranging thread to update the count for polling neighbors and suspending/releasing the responder task independently from the polling rate.
> 
> > **Type:** `int`
> >
> > **Defaults:** `100`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_PWRAMP_ALLOW_BAD_CHANNELS
> _Allow invalid UWB channels to use the amplifier_
> 
> > **Type:** `bool`
> >
> > **Defaults:** `n`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_REPORT_UWB_DROPS
> _Report dropped UWB ranging estimates_
> 
> > **Type:** `bool`
> >
> > **Defaults:** `n`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_UWB_DIAGNOSTICS
> _Record UWB diagnostics with the ranging estimates_
> 
> This allows the DW1000 to record its diagnostics information after each successful ranging attempt. WARNING: The more nodes that are on the network, the more heap memory is required to report the data. It is recommended that at most 4 nodes are used.
> 
> > **Type:** `bool`
> >
> > **Defaults:** `n`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_UWB_FRAME_FILTER
> _Filter frames on the DW1000 [Experimental]_
> 
> > **Type:** `bool`
> >
> > **Defaults:** `n`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_RESPONDER_TIMEOUT
> _UWB Responder task timeout (ms)_
> 
> > **Type:** `int`
> >
> > **Defaults:** `1000`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_INITIATOR_TIMEOUT
> _UWB Initiator task timeout (ms)_
> 
> > **Type:** `int`
> >
> > **Defaults:** `1000`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`

## CONFIG_RESP_SUSPEND_SCHED
> _Suspend scheduler when responding [EXPERIMENTAL]_
> 
> > **Type:** `bool`
> >
> > **Defaults:** `n`
> >
> > **Menu Path:** `(Top) > Beluga Configurations`
