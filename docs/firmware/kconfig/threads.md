## CONFIG_ENABLE_COMMANDS
> _Enable commands for Beluga_
>
> > **Type:** `bool`
> >
> > **Defaults:** `y`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations`

## CONFIG_BELUGA_COMMANDS_PRIO
> _Priority for commands thread_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_COMMANDS](threads.md#config_enable_commands)
> >
> > **Defaults:** `7`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable commands for Beluga`

## CONFIG_COMMANDS_STACK_SIZE
> _Stack size (bytes) for commands thread_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_COMMANDS](threads.md#config_enable_commands)
> >
> > **Defaults:** `2048`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable commands for Beluga`

## CONFIG_ENABLE_LIST
> _Enable the list thread for Beluga_
>
> > **Type:** `bool`
> >
> > **Defaults:** `y`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations`

## CONFIG_BELUGA_LIST_PRIO
> _Priority for list thread_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_LIST](threads.md#config_enable_list)
> >
> > **Defaults:** `7`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable the list thread for Beluga`

## CONFIG_LIST_STACK_SIZE
> _Stack size (bytes) for list thread_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_LIST](threads.md#config_enable_list)
> >
> > **Defaults:** `3640`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable the list thread for Beluga`

## CONFIG_ENABLE_RANGING
> _Enable the ranging thread for Beluga_
>
> > **Type:** `bool`
> >
> > **Defaults:** `y`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations`

## CONFIG_BELUGA_RANGING_PRIO
> _Priority for ranging thread_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_RANGING](threads.md#config_enable_ranging)
> >
> > **Defaults:** `7`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable the ranging thread for Beluga`

## CONFIG_RANGING_STACK_SIZE
> _Stack size (bytes) for ranging thread_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_RANGING](threads.md#config_enable_ranging)
> >
> > **Defaults:** `1240`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable the ranging thread for Beluga`

## CONFIG_ENABLE_MONITOR
> _Enable the monitor thread in Beluga_
>
> > **Type:** `bool`
> >
> > **Defaults:** `y`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations`

## CONFIG_BELUGA_MONITOR_PRIO
> _Priority for monitor thread_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_MONITOR](threads.md#config_enable_monitor)
> >
> > **Defaults:** `7`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable the monitor thread in Beluga`

## CONFIG_MONITOR_STACK_SIZE
> _Stack size (bytes) for monitor task_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_MONITOR](threads.md#config_enable_monitor)
> >
> > **Defaults:** `2940`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable the monitor thread in Beluga`

## CONFIG_ENABLE_RESPONDER
> _Enable the responder thread in Beluga_
>
> > **Type:** `bool`
> >
> > **Defaults:** `y`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations`

## CONFIG_BELUGA_RESPONDER_PRIO
> _Priority for responder thread_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_RESPONDER](threads.md#config_enable_responder)
> >
> > **Defaults:** `8`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable the responder thread in Beluga`

## CONFIG_RESPONDER_STACK_SIZE
> _Stack size (bytes) for responder task_
>
> > **Type:** `int`
> >
> > **Depends:** [CONFIG_ENABLE_RESPONDER](threads.md#config_enable_responder)
> >
> > **Defaults:** `1536`
> >
> > **Menu Path:** `(Top) > Beluga Configurations > Thread Configurations > Enable the responder thread in Beluga`
