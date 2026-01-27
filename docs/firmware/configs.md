## Software Configurations

### Logging

#### CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_client_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client`

#### CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_client_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client`

#### CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_client_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client`

#### CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_client_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client`

#### CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_client_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client`

#### CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_client_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client`

#### CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Beluga Service Client_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_DEFAULT](configs.md#config_beluga_service_client_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_DBG](configs.md#config_beluga_service_client_log_level_dbg)
> > - [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_INF](configs.md#config_beluga_service_client_log_level_inf)
> > - [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_WRN](configs.md#config_beluga_service_client_log_level_wrn)
> > - [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_ERR](configs.md#config_beluga_service_client_log_level_err)
> > - [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_OFF](configs.md#config_beluga_service_client_log_level_off)
> > - [CONFIG_BELUGA_SERVICE_CLIENT_LOG_LEVEL_DEFAULT](configs.md#config_beluga_service_client_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client`

#### CONFIG_BELUGA_SERVICE_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service`

#### CONFIG_BELUGA_SERVICE_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service`

#### CONFIG_BELUGA_SERVICE_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service`

#### CONFIG_BELUGA_SERVICE_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service`

#### CONFIG_BELUGA_SERVICE_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service`

#### CONFIG_BELUGA_SERVICE_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_SERVICE_LOG_LEVEL_CHOICE](configs.md#config_beluga_service_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service`

#### CONFIG_BELUGA_SERVICE_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Beluga Service_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_BELUGA_SERVICE_LOG_LEVEL_DEFAULT](configs.md#config_beluga_service_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_BELUGA_SERVICE_LOG_LEVEL_DBG](configs.md#config_beluga_service_log_level_dbg)
> > - [CONFIG_BELUGA_SERVICE_LOG_LEVEL_INF](configs.md#config_beluga_service_log_level_inf)
> > - [CONFIG_BELUGA_SERVICE_LOG_LEVEL_WRN](configs.md#config_beluga_service_log_level_wrn)
> > - [CONFIG_BELUGA_SERVICE_LOG_LEVEL_ERR](configs.md#config_beluga_service_log_level_err)
> > - [CONFIG_BELUGA_SERVICE_LOG_LEVEL_OFF](configs.md#config_beluga_service_log_level_off)
> > - [CONFIG_BELUGA_SERVICE_LOG_LEVEL_DEFAULT](configs.md#config_beluga_service_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service`

#### CONFIG_COMMS_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_LOG_LEVEL_CHOICE](configs.md#config_comms_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms`

#### CONFIG_COMMS_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_LOG_LEVEL_CHOICE](configs.md#config_comms_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms`

#### CONFIG_COMMS_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_LOG_LEVEL_CHOICE](configs.md#config_comms_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms`

#### CONFIG_COMMS_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_LOG_LEVEL_CHOICE](configs.md#config_comms_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms`

#### CONFIG_COMMS_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_LOG_LEVEL_CHOICE](configs.md#config_comms_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms`

#### CONFIG_COMMS_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_LOG_LEVEL_CHOICE](configs.md#config_comms_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms`

#### CONFIG_COMMS_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Comms module_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_COMMS_LOG_LEVEL_DEFAULT](configs.md#config_comms_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_COMMS_LOG_LEVEL_DBG](configs.md#config_comms_log_level_dbg)
> > - [CONFIG_COMMS_LOG_LEVEL_INF](configs.md#config_comms_log_level_inf)
> > - [CONFIG_COMMS_LOG_LEVEL_WRN](configs.md#config_comms_log_level_wrn)
> > - [CONFIG_COMMS_LOG_LEVEL_ERR](configs.md#config_comms_log_level_err)
> > - [CONFIG_COMMS_LOG_LEVEL_OFF](configs.md#config_comms_log_level_off)
> > - [CONFIG_COMMS_LOG_LEVEL_DEFAULT](configs.md#config_comms_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms`

#### CONFIG_COMMS_SERIAL_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_SERIAL_LOG_LEVEL_CHOICE](configs.md#config_comms_serial_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend`

#### CONFIG_COMMS_SERIAL_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_SERIAL_LOG_LEVEL_CHOICE](configs.md#config_comms_serial_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend`

#### CONFIG_COMMS_SERIAL_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_SERIAL_LOG_LEVEL_CHOICE](configs.md#config_comms_serial_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend`

#### CONFIG_COMMS_SERIAL_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_SERIAL_LOG_LEVEL_CHOICE](configs.md#config_comms_serial_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend`

#### CONFIG_COMMS_SERIAL_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_SERIAL_LOG_LEVEL_CHOICE](configs.md#config_comms_serial_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend`

#### CONFIG_COMMS_SERIAL_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_COMMS_SERIAL_LOG_LEVEL_CHOICE](configs.md#config_comms_serial_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend`

#### CONFIG_COMMS_SERIAL_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Comms Backend module_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_COMMS_SERIAL_LOG_LEVEL_DEFAULT](configs.md#config_comms_serial_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_COMMS_SERIAL_LOG_LEVEL_DBG](configs.md#config_comms_serial_log_level_dbg)
> > - [CONFIG_COMMS_SERIAL_LOG_LEVEL_INF](configs.md#config_comms_serial_log_level_inf)
> > - [CONFIG_COMMS_SERIAL_LOG_LEVEL_WRN](configs.md#config_comms_serial_log_level_wrn)
> > - [CONFIG_COMMS_SERIAL_LOG_LEVEL_ERR](configs.md#config_comms_serial_log_level_err)
> > - [CONFIG_COMMS_SERIAL_LOG_LEVEL_OFF](configs.md#config_comms_serial_log_level_off)
> > - [CONFIG_COMMS_SERIAL_LOG_LEVEL_DEFAULT](configs.md#config_comms_serial_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend`

#### CONFIG_APP_LEDS_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_APP_LEDS_LOG_LEVEL_CHOICE](configs.md#config_app_leds_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs`

#### CONFIG_APP_LEDS_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_APP_LEDS_LOG_LEVEL_CHOICE](configs.md#config_app_leds_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs`

#### CONFIG_APP_LEDS_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_APP_LEDS_LOG_LEVEL_CHOICE](configs.md#config_app_leds_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs`

#### CONFIG_APP_LEDS_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_APP_LEDS_LOG_LEVEL_CHOICE](configs.md#config_app_leds_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs`

#### CONFIG_APP_LEDS_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_APP_LEDS_LOG_LEVEL_CHOICE](configs.md#config_app_leds_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs`

#### CONFIG_APP_LEDS_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_APP_LEDS_LOG_LEVEL_CHOICE](configs.md#config_app_leds_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs`

#### CONFIG_APP_LEDS_LOG_LEVEL_CHOICE
> _Max compiled-in log level for App LEDs_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_APP_LEDS_LOG_LEVEL_DEFAULT](configs.md#config_app_leds_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_APP_LEDS_LOG_LEVEL_DBG](configs.md#config_app_leds_log_level_dbg)
> > - [CONFIG_APP_LEDS_LOG_LEVEL_INF](configs.md#config_app_leds_log_level_inf)
> > - [CONFIG_APP_LEDS_LOG_LEVEL_WRN](configs.md#config_app_leds_log_level_wrn)
> > - [CONFIG_APP_LEDS_LOG_LEVEL_ERR](configs.md#config_app_leds_log_level_err)
> > - [CONFIG_APP_LEDS_LOG_LEVEL_OFF](configs.md#config_app_leds_log_level_off)
> > - [CONFIG_APP_LEDS_LOG_LEVEL_DEFAULT](configs.md#config_app_leds_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs`

#### CONFIG_AT_COMMANDS_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_AT_COMMANDS_LOG_LEVEL_CHOICE](configs.md#config_at_commands_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands`

#### CONFIG_AT_COMMANDS_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_AT_COMMANDS_LOG_LEVEL_CHOICE](configs.md#config_at_commands_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands`

#### CONFIG_AT_COMMANDS_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_AT_COMMANDS_LOG_LEVEL_CHOICE](configs.md#config_at_commands_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands`

#### CONFIG_AT_COMMANDS_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_AT_COMMANDS_LOG_LEVEL_CHOICE](configs.md#config_at_commands_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands`

#### CONFIG_AT_COMMANDS_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_AT_COMMANDS_LOG_LEVEL_CHOICE](configs.md#config_at_commands_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands`

#### CONFIG_AT_COMMANDS_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_AT_COMMANDS_LOG_LEVEL_CHOICE](configs.md#config_at_commands_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands`

#### CONFIG_AT_COMMANDS_LOG_LEVEL_CHOICE
> _Max compiled-in log level for AT commands_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_AT_COMMANDS_LOG_LEVEL_DEFAULT](configs.md#config_at_commands_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_AT_COMMANDS_LOG_LEVEL_DBG](configs.md#config_at_commands_log_level_dbg)
> > - [CONFIG_AT_COMMANDS_LOG_LEVEL_INF](configs.md#config_at_commands_log_level_inf)
> > - [CONFIG_AT_COMMANDS_LOG_LEVEL_WRN](configs.md#config_at_commands_log_level_wrn)
> > - [CONFIG_AT_COMMANDS_LOG_LEVEL_ERR](configs.md#config_at_commands_log_level_err)
> > - [CONFIG_AT_COMMANDS_LOG_LEVEL_OFF](configs.md#config_at_commands_log_level_off)
> > - [CONFIG_AT_COMMANDS_LOG_LEVEL_DEFAULT](configs.md#config_at_commands_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands`

#### CONFIG_BELUGA_MESSAGE_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_CHOICE](configs.md#config_beluga_message_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message`

#### CONFIG_BELUGA_MESSAGE_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_CHOICE](configs.md#config_beluga_message_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message`

#### CONFIG_BELUGA_MESSAGE_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_CHOICE](configs.md#config_beluga_message_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message`

#### CONFIG_BELUGA_MESSAGE_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_CHOICE](configs.md#config_beluga_message_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message`

#### CONFIG_BELUGA_MESSAGE_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_CHOICE](configs.md#config_beluga_message_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message`

#### CONFIG_BELUGA_MESSAGE_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_CHOICE](configs.md#config_beluga_message_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message`

#### CONFIG_BELUGA_MESSAGE_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Beluga message_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_DEFAULT](configs.md#config_beluga_message_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_DBG](configs.md#config_beluga_message_log_level_dbg)
> > - [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_INF](configs.md#config_beluga_message_log_level_inf)
> > - [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_WRN](configs.md#config_beluga_message_log_level_wrn)
> > - [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_ERR](configs.md#config_beluga_message_log_level_err)
> > - [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_OFF](configs.md#config_beluga_message_log_level_off)
> > - [CONFIG_BELUGA_MESSAGE_LOG_LEVEL_DEFAULT](configs.md#config_beluga_message_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message`

#### CONFIG_BLE_APP_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BLE_APP_LOG_LEVEL_CHOICE](configs.md#config_ble_app_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application`

#### CONFIG_BLE_APP_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BLE_APP_LOG_LEVEL_CHOICE](configs.md#config_ble_app_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application`

#### CONFIG_BLE_APP_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BLE_APP_LOG_LEVEL_CHOICE](configs.md#config_ble_app_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application`

#### CONFIG_BLE_APP_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BLE_APP_LOG_LEVEL_CHOICE](configs.md#config_ble_app_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application`

#### CONFIG_BLE_APP_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BLE_APP_LOG_LEVEL_CHOICE](configs.md#config_ble_app_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application`

#### CONFIG_BLE_APP_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BLE_APP_LOG_LEVEL_CHOICE](configs.md#config_ble_app_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application`

#### CONFIG_BLE_APP_LOG_LEVEL_CHOICE
> _Max compiled-in log level for BLE application_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_BLE_APP_LOG_LEVEL_DEFAULT](configs.md#config_ble_app_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_BLE_APP_LOG_LEVEL_DBG](configs.md#config_ble_app_log_level_dbg)
> > - [CONFIG_BLE_APP_LOG_LEVEL_INF](configs.md#config_ble_app_log_level_inf)
> > - [CONFIG_BLE_APP_LOG_LEVEL_WRN](configs.md#config_ble_app_log_level_wrn)
> > - [CONFIG_BLE_APP_LOG_LEVEL_ERR](configs.md#config_ble_app_log_level_err)
> > - [CONFIG_BLE_APP_LOG_LEVEL_OFF](configs.md#config_ble_app_log_level_off)
> > - [CONFIG_BLE_APP_LOG_LEVEL_DEFAULT](configs.md#config_ble_app_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application`

#### CONFIG_BELUGA_DEBUG_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_DEBUG_LOG_LEVEL_CHOICE](configs.md#config_beluga_debug_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug`

#### CONFIG_BELUGA_DEBUG_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_DEBUG_LOG_LEVEL_CHOICE](configs.md#config_beluga_debug_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug`

#### CONFIG_BELUGA_DEBUG_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_DEBUG_LOG_LEVEL_CHOICE](configs.md#config_beluga_debug_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug`

#### CONFIG_BELUGA_DEBUG_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_DEBUG_LOG_LEVEL_CHOICE](configs.md#config_beluga_debug_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug`

#### CONFIG_BELUGA_DEBUG_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_DEBUG_LOG_LEVEL_CHOICE](configs.md#config_beluga_debug_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug`

#### CONFIG_BELUGA_DEBUG_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_DEBUG_LOG_LEVEL_CHOICE](configs.md#config_beluga_debug_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug`

#### CONFIG_BELUGA_DEBUG_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Beluga debugging_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_BELUGA_DEBUG_LOG_LEVEL_DEFAULT](configs.md#config_beluga_debug_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_BELUGA_DEBUG_LOG_LEVEL_DBG](configs.md#config_beluga_debug_log_level_dbg)
> > - [CONFIG_BELUGA_DEBUG_LOG_LEVEL_INF](configs.md#config_beluga_debug_log_level_inf)
> > - [CONFIG_BELUGA_DEBUG_LOG_LEVEL_WRN](configs.md#config_beluga_debug_log_level_wrn)
> > - [CONFIG_BELUGA_DEBUG_LOG_LEVEL_ERR](configs.md#config_beluga_debug_log_level_err)
> > - [CONFIG_BELUGA_DEBUG_LOG_LEVEL_OFF](configs.md#config_beluga_debug_log_level_off)
> > - [CONFIG_BELUGA_DEBUG_LOG_LEVEL_DEFAULT](configs.md#config_beluga_debug_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug`

#### CONFIG_INITIATOR_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_INITIATOR_LOG_LEVEL_CHOICE](configs.md#config_initiator_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator`

#### CONFIG_INITIATOR_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_INITIATOR_LOG_LEVEL_CHOICE](configs.md#config_initiator_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator`

#### CONFIG_INITIATOR_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_INITIATOR_LOG_LEVEL_CHOICE](configs.md#config_initiator_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator`

#### CONFIG_INITIATOR_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_INITIATOR_LOG_LEVEL_CHOICE](configs.md#config_initiator_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator`

#### CONFIG_INITIATOR_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_INITIATOR_LOG_LEVEL_CHOICE](configs.md#config_initiator_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator`

#### CONFIG_INITIATOR_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_INITIATOR_LOG_LEVEL_CHOICE](configs.md#config_initiator_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator`

#### CONFIG_INITIATOR_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Initiator_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_INITIATOR_LOG_LEVEL_DEFAULT](configs.md#config_initiator_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_INITIATOR_LOG_LEVEL_DBG](configs.md#config_initiator_log_level_dbg)
> > - [CONFIG_INITIATOR_LOG_LEVEL_INF](configs.md#config_initiator_log_level_inf)
> > - [CONFIG_INITIATOR_LOG_LEVEL_WRN](configs.md#config_initiator_log_level_wrn)
> > - [CONFIG_INITIATOR_LOG_LEVEL_ERR](configs.md#config_initiator_log_level_err)
> > - [CONFIG_INITIATOR_LOG_LEVEL_OFF](configs.md#config_initiator_log_level_off)
> > - [CONFIG_INITIATOR_LOG_LEVEL_DEFAULT](configs.md#config_initiator_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator`

#### CONFIG_LIST_MONITOR_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LIST_MONITOR_LOG_LEVEL_CHOICE](configs.md#config_list_monitor_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor`

#### CONFIG_LIST_MONITOR_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LIST_MONITOR_LOG_LEVEL_CHOICE](configs.md#config_list_monitor_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor`

#### CONFIG_LIST_MONITOR_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LIST_MONITOR_LOG_LEVEL_CHOICE](configs.md#config_list_monitor_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor`

#### CONFIG_LIST_MONITOR_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LIST_MONITOR_LOG_LEVEL_CHOICE](configs.md#config_list_monitor_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor`

#### CONFIG_LIST_MONITOR_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LIST_MONITOR_LOG_LEVEL_CHOICE](configs.md#config_list_monitor_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor`

#### CONFIG_LIST_MONITOR_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LIST_MONITOR_LOG_LEVEL_CHOICE](configs.md#config_list_monitor_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor`

#### CONFIG_LIST_MONITOR_LOG_LEVEL_CHOICE
> _Max compiled-in log level for List monitor_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_LIST_MONITOR_LOG_LEVEL_DEFAULT](configs.md#config_list_monitor_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_LIST_MONITOR_LOG_LEVEL_DBG](configs.md#config_list_monitor_log_level_dbg)
> > - [CONFIG_LIST_MONITOR_LOG_LEVEL_INF](configs.md#config_list_monitor_log_level_inf)
> > - [CONFIG_LIST_MONITOR_LOG_LEVEL_WRN](configs.md#config_list_monitor_log_level_wrn)
> > - [CONFIG_LIST_MONITOR_LOG_LEVEL_ERR](configs.md#config_list_monitor_log_level_err)
> > - [CONFIG_LIST_MONITOR_LOG_LEVEL_OFF](configs.md#config_list_monitor_log_level_off)
> > - [CONFIG_LIST_MONITOR_LOG_LEVEL_DEFAULT](configs.md#config_list_monitor_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor`

#### CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_CHOICE](configs.md#config_neighbor_listing_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing`

#### CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_CHOICE](configs.md#config_neighbor_listing_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing`

#### CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_CHOICE](configs.md#config_neighbor_listing_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing`

#### CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_CHOICE](configs.md#config_neighbor_listing_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing`

#### CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_CHOICE](configs.md#config_neighbor_listing_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing`

#### CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_CHOICE](configs.md#config_neighbor_listing_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing`

#### CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Neighbor listing module_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_DEFAULT](configs.md#config_neighbor_listing_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_DBG](configs.md#config_neighbor_listing_log_level_dbg)
> > - [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_INF](configs.md#config_neighbor_listing_log_level_inf)
> > - [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_WRN](configs.md#config_neighbor_listing_log_level_wrn)
> > - [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_ERR](configs.md#config_neighbor_listing_log_level_err)
> > - [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_OFF](configs.md#config_neighbor_listing_log_level_off)
> > - [CONFIG_NEIGHBOR_LISTING_LOG_LEVEL_DEFAULT](configs.md#config_neighbor_listing_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing`

#### CONFIG_BELUGA_MAIN_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MAIN_LOG_LEVEL_CHOICE](configs.md#config_beluga_main_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application`

#### CONFIG_BELUGA_MAIN_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MAIN_LOG_LEVEL_CHOICE](configs.md#config_beluga_main_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application`

#### CONFIG_BELUGA_MAIN_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MAIN_LOG_LEVEL_CHOICE](configs.md#config_beluga_main_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application`

#### CONFIG_BELUGA_MAIN_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MAIN_LOG_LEVEL_CHOICE](configs.md#config_beluga_main_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application`

#### CONFIG_BELUGA_MAIN_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MAIN_LOG_LEVEL_CHOICE](configs.md#config_beluga_main_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application`

#### CONFIG_BELUGA_MAIN_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_BELUGA_MAIN_LOG_LEVEL_CHOICE](configs.md#config_beluga_main_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application`

#### CONFIG_BELUGA_MAIN_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Beluga main_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_BELUGA_MAIN_LOG_LEVEL_DEFAULT](configs.md#config_beluga_main_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_BELUGA_MAIN_LOG_LEVEL_DBG](configs.md#config_beluga_main_log_level_dbg)
> > - [CONFIG_BELUGA_MAIN_LOG_LEVEL_INF](configs.md#config_beluga_main_log_level_inf)
> > - [CONFIG_BELUGA_MAIN_LOG_LEVEL_WRN](configs.md#config_beluga_main_log_level_wrn)
> > - [CONFIG_BELUGA_MAIN_LOG_LEVEL_ERR](configs.md#config_beluga_main_log_level_err)
> > - [CONFIG_BELUGA_MAIN_LOG_LEVEL_OFF](configs.md#config_beluga_main_log_level_off)
> > - [CONFIG_BELUGA_MAIN_LOG_LEVEL_DEFAULT](configs.md#config_beluga_main_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application`

#### CONFIG_POWER_MANAGER_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_POWER_MANAGER_LOG_LEVEL_CHOICE](configs.md#config_power_manager_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager`

#### CONFIG_POWER_MANAGER_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_POWER_MANAGER_LOG_LEVEL_CHOICE](configs.md#config_power_manager_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager`

#### CONFIG_POWER_MANAGER_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_POWER_MANAGER_LOG_LEVEL_CHOICE](configs.md#config_power_manager_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager`

#### CONFIG_POWER_MANAGER_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_POWER_MANAGER_LOG_LEVEL_CHOICE](configs.md#config_power_manager_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager`

#### CONFIG_POWER_MANAGER_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_POWER_MANAGER_LOG_LEVEL_CHOICE](configs.md#config_power_manager_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager`

#### CONFIG_POWER_MANAGER_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_POWER_MANAGER_LOG_LEVEL_CHOICE](configs.md#config_power_manager_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager`

#### CONFIG_POWER_MANAGER_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Power manager_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_POWER_MANAGER_LOG_LEVEL_DEFAULT](configs.md#config_power_manager_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_POWER_MANAGER_LOG_LEVEL_DBG](configs.md#config_power_manager_log_level_dbg)
> > - [CONFIG_POWER_MANAGER_LOG_LEVEL_INF](configs.md#config_power_manager_log_level_inf)
> > - [CONFIG_POWER_MANAGER_LOG_LEVEL_WRN](configs.md#config_power_manager_log_level_wrn)
> > - [CONFIG_POWER_MANAGER_LOG_LEVEL_ERR](configs.md#config_power_manager_log_level_err)
> > - [CONFIG_POWER_MANAGER_LOG_LEVEL_OFF](configs.md#config_power_manager_log_level_off)
> > - [CONFIG_POWER_MANAGER_LOG_LEVEL_DEFAULT](configs.md#config_power_manager_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager`

#### CONFIG_RANGE_EXTENSION_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGE_EXTENSION_LOG_LEVEL_CHOICE](configs.md#config_range_extension_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension`

#### CONFIG_RANGE_EXTENSION_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGE_EXTENSION_LOG_LEVEL_CHOICE](configs.md#config_range_extension_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension`

#### CONFIG_RANGE_EXTENSION_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGE_EXTENSION_LOG_LEVEL_CHOICE](configs.md#config_range_extension_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension`

#### CONFIG_RANGE_EXTENSION_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGE_EXTENSION_LOG_LEVEL_CHOICE](configs.md#config_range_extension_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension`

#### CONFIG_RANGE_EXTENSION_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGE_EXTENSION_LOG_LEVEL_CHOICE](configs.md#config_range_extension_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension`

#### CONFIG_RANGE_EXTENSION_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGE_EXTENSION_LOG_LEVEL_CHOICE](configs.md#config_range_extension_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension`

#### CONFIG_RANGE_EXTENSION_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Range extension_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_RANGE_EXTENSION_LOG_LEVEL_DEFAULT](configs.md#config_range_extension_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_RANGE_EXTENSION_LOG_LEVEL_DBG](configs.md#config_range_extension_log_level_dbg)
> > - [CONFIG_RANGE_EXTENSION_LOG_LEVEL_INF](configs.md#config_range_extension_log_level_inf)
> > - [CONFIG_RANGE_EXTENSION_LOG_LEVEL_WRN](configs.md#config_range_extension_log_level_wrn)
> > - [CONFIG_RANGE_EXTENSION_LOG_LEVEL_ERR](configs.md#config_range_extension_log_level_err)
> > - [CONFIG_RANGE_EXTENSION_LOG_LEVEL_OFF](configs.md#config_range_extension_log_level_off)
> > - [CONFIG_RANGE_EXTENSION_LOG_LEVEL_DEFAULT](configs.md#config_range_extension_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension`

#### CONFIG_RANGING_MODULE_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGING_MODULE_LOG_LEVEL_CHOICE](configs.md#config_ranging_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging`

#### CONFIG_RANGING_MODULE_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGING_MODULE_LOG_LEVEL_CHOICE](configs.md#config_ranging_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging`

#### CONFIG_RANGING_MODULE_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGING_MODULE_LOG_LEVEL_CHOICE](configs.md#config_ranging_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging`

#### CONFIG_RANGING_MODULE_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGING_MODULE_LOG_LEVEL_CHOICE](configs.md#config_ranging_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging`

#### CONFIG_RANGING_MODULE_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGING_MODULE_LOG_LEVEL_CHOICE](configs.md#config_ranging_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging`

#### CONFIG_RANGING_MODULE_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RANGING_MODULE_LOG_LEVEL_CHOICE](configs.md#config_ranging_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging`

#### CONFIG_RANGING_MODULE_LOG_LEVEL_CHOICE
> _Max compiled-in log level for ranging module_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_RANGING_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_ranging_module_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_RANGING_MODULE_LOG_LEVEL_DBG](configs.md#config_ranging_module_log_level_dbg)
> > - [CONFIG_RANGING_MODULE_LOG_LEVEL_INF](configs.md#config_ranging_module_log_level_inf)
> > - [CONFIG_RANGING_MODULE_LOG_LEVEL_WRN](configs.md#config_ranging_module_log_level_wrn)
> > - [CONFIG_RANGING_MODULE_LOG_LEVEL_ERR](configs.md#config_ranging_module_log_level_err)
> > - [CONFIG_RANGING_MODULE_LOG_LEVEL_OFF](configs.md#config_ranging_module_log_level_off)
> > - [CONFIG_RANGING_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_ranging_module_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging`

#### CONFIG_RESPONDER_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RESPONDER_LOG_LEVEL_CHOICE](configs.md#config_responder_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder`

#### CONFIG_RESPONDER_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RESPONDER_LOG_LEVEL_CHOICE](configs.md#config_responder_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder`

#### CONFIG_RESPONDER_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RESPONDER_LOG_LEVEL_CHOICE](configs.md#config_responder_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder`

#### CONFIG_RESPONDER_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RESPONDER_LOG_LEVEL_CHOICE](configs.md#config_responder_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder`

#### CONFIG_RESPONDER_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RESPONDER_LOG_LEVEL_CHOICE](configs.md#config_responder_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder`

#### CONFIG_RESPONDER_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_RESPONDER_LOG_LEVEL_CHOICE](configs.md#config_responder_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder`

#### CONFIG_RESPONDER_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Responder_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_RESPONDER_LOG_LEVEL_DEFAULT](configs.md#config_responder_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_RESPONDER_LOG_LEVEL_DBG](configs.md#config_responder_log_level_dbg)
> > - [CONFIG_RESPONDER_LOG_LEVEL_INF](configs.md#config_responder_log_level_inf)
> > - [CONFIG_RESPONDER_LOG_LEVEL_WRN](configs.md#config_responder_log_level_wrn)
> > - [CONFIG_RESPONDER_LOG_LEVEL_ERR](configs.md#config_responder_log_level_err)
> > - [CONFIG_RESPONDER_LOG_LEVEL_OFF](configs.md#config_responder_log_level_off)
> > - [CONFIG_RESPONDER_LOG_LEVEL_DEFAULT](configs.md#config_responder_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder`

#### CONFIG_SERIAL_LED_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SERIAL_LED_LOG_LEVEL_CHOICE](configs.md#config_serial_led_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs`

#### CONFIG_SERIAL_LED_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SERIAL_LED_LOG_LEVEL_CHOICE](configs.md#config_serial_led_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs`

#### CONFIG_SERIAL_LED_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SERIAL_LED_LOG_LEVEL_CHOICE](configs.md#config_serial_led_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs`

#### CONFIG_SERIAL_LED_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SERIAL_LED_LOG_LEVEL_CHOICE](configs.md#config_serial_led_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs`

#### CONFIG_SERIAL_LED_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SERIAL_LED_LOG_LEVEL_CHOICE](configs.md#config_serial_led_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs`

#### CONFIG_SERIAL_LED_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SERIAL_LED_LOG_LEVEL_CHOICE](configs.md#config_serial_led_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs`

#### CONFIG_SERIAL_LED_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Serial LEDs_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_SERIAL_LED_LOG_LEVEL_DEFAULT](configs.md#config_serial_led_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_SERIAL_LED_LOG_LEVEL_DBG](configs.md#config_serial_led_log_level_dbg)
> > - [CONFIG_SERIAL_LED_LOG_LEVEL_INF](configs.md#config_serial_led_log_level_inf)
> > - [CONFIG_SERIAL_LED_LOG_LEVEL_WRN](configs.md#config_serial_led_log_level_wrn)
> > - [CONFIG_SERIAL_LED_LOG_LEVEL_ERR](configs.md#config_serial_led_log_level_err)
> > - [CONFIG_SERIAL_LED_LOG_LEVEL_OFF](configs.md#config_serial_led_log_level_off)
> > - [CONFIG_SERIAL_LED_LOG_LEVEL_DEFAULT](configs.md#config_serial_led_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs`

#### CONFIG_SETTINGS_MODULE_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SETTINGS_MODULE_LOG_LEVEL_CHOICE](configs.md#config_settings_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings`

#### CONFIG_SETTINGS_MODULE_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SETTINGS_MODULE_LOG_LEVEL_CHOICE](configs.md#config_settings_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings`

#### CONFIG_SETTINGS_MODULE_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SETTINGS_MODULE_LOG_LEVEL_CHOICE](configs.md#config_settings_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings`

#### CONFIG_SETTINGS_MODULE_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SETTINGS_MODULE_LOG_LEVEL_CHOICE](configs.md#config_settings_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings`

#### CONFIG_SETTINGS_MODULE_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SETTINGS_MODULE_LOG_LEVEL_CHOICE](configs.md#config_settings_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings`

#### CONFIG_SETTINGS_MODULE_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SETTINGS_MODULE_LOG_LEVEL_CHOICE](configs.md#config_settings_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings`

#### CONFIG_SETTINGS_MODULE_LOG_LEVEL_CHOICE
> _Max compiled-in log level for settings_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_SETTINGS_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_settings_module_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_SETTINGS_MODULE_LOG_LEVEL_DBG](configs.md#config_settings_module_log_level_dbg)
> > - [CONFIG_SETTINGS_MODULE_LOG_LEVEL_INF](configs.md#config_settings_module_log_level_inf)
> > - [CONFIG_SETTINGS_MODULE_LOG_LEVEL_WRN](configs.md#config_settings_module_log_level_wrn)
> > - [CONFIG_SETTINGS_MODULE_LOG_LEVEL_ERR](configs.md#config_settings_module_log_level_err)
> > - [CONFIG_SETTINGS_MODULE_LOG_LEVEL_OFF](configs.md#config_settings_module_log_level_off)
> > - [CONFIG_SETTINGS_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_settings_module_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings`

#### CONFIG_SPI_MODULE_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SPI_MODULE_LOG_LEVEL_CHOICE](configs.md#config_spi_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module`

#### CONFIG_SPI_MODULE_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SPI_MODULE_LOG_LEVEL_CHOICE](configs.md#config_spi_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module`

#### CONFIG_SPI_MODULE_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SPI_MODULE_LOG_LEVEL_CHOICE](configs.md#config_spi_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module`

#### CONFIG_SPI_MODULE_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SPI_MODULE_LOG_LEVEL_CHOICE](configs.md#config_spi_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module`

#### CONFIG_SPI_MODULE_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SPI_MODULE_LOG_LEVEL_CHOICE](configs.md#config_spi_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module`

#### CONFIG_SPI_MODULE_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_SPI_MODULE_LOG_LEVEL_CHOICE](configs.md#config_spi_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module`

#### CONFIG_SPI_MODULE_LOG_LEVEL_CHOICE
> _Max compiled-in log level for SPI module_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_SPI_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_spi_module_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_SPI_MODULE_LOG_LEVEL_DBG](configs.md#config_spi_module_log_level_dbg)
> > - [CONFIG_SPI_MODULE_LOG_LEVEL_INF](configs.md#config_spi_module_log_level_inf)
> > - [CONFIG_SPI_MODULE_LOG_LEVEL_WRN](configs.md#config_spi_module_log_level_wrn)
> > - [CONFIG_SPI_MODULE_LOG_LEVEL_ERR](configs.md#config_spi_module_log_level_err)
> > - [CONFIG_SPI_MODULE_LOG_LEVEL_OFF](configs.md#config_spi_module_log_level_off)
> > - [CONFIG_SPI_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_spi_module_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module`

#### CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_voltage_reg_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator`

#### CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_voltage_reg_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator`

#### CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_voltage_reg_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator`

#### CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_voltage_reg_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator`

#### CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_voltage_reg_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator`

#### CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_voltage_reg_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator`

#### CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_CHOICE
> _Max compiled-in log level for Voltage regulator module_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_voltage_reg_module_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_DBG](configs.md#config_voltage_reg_module_log_level_dbg)
> > - [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_INF](configs.md#config_voltage_reg_module_log_level_inf)
> > - [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_WRN](configs.md#config_voltage_reg_module_log_level_wrn)
> > - [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_ERR](configs.md#config_voltage_reg_module_log_level_err)
> > - [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_OFF](configs.md#config_voltage_reg_module_log_level_off)
> > - [CONFIG_VOLTAGE_REG_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_voltage_reg_module_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator`

#### CONFIG_WATCHDOG_MODULE_LOG_LEVEL_DBG
> _Debug_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_watchdog_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator > Watchdog`

#### CONFIG_WATCHDOG_MODULE_LOG_LEVEL_INF
> _Info_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_watchdog_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator > Watchdog`

#### CONFIG_WATCHDOG_MODULE_LOG_LEVEL_WRN
> _Warning_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_watchdog_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator > Watchdog`

#### CONFIG_WATCHDOG_MODULE_LOG_LEVEL_ERR
> _Error_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_watchdog_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator > Watchdog`

#### CONFIG_WATCHDOG_MODULE_LOG_LEVEL_OFF
> _Off_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_watchdog_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator > Watchdog`

#### CONFIG_WATCHDOG_MODULE_LOG_LEVEL_DEFAULT
> _Default_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_CHOICE](configs.md#config_watchdog_module_log_level_choice)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator > Watchdog`

#### CONFIG_WATCHDOG_MODULE_LOG_LEVEL_CHOICE
> _Max compiled-in log level for watchdog module_
>
> > **Type:** `bool`
> >
> > **Dependencies:** [CONFIG_LOG](https://docs.zephyrproject.org/latest/kconfig.html#CONFIG_LOG){target="_blank"}
> >
> > **Defaults:** [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_watchdog_module_log_level_default)
> >
> > **Choices:**
> >
> > - [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_DBG](configs.md#config_watchdog_module_log_level_dbg)
> > - [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_INF](configs.md#config_watchdog_module_log_level_inf)
> > - [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_WRN](configs.md#config_watchdog_module_log_level_wrn)
> > - [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_ERR](configs.md#config_watchdog_module_log_level_err)
> > - [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_OFF](configs.md#config_watchdog_module_log_level_off)
> > - [CONFIG_WATCHDOG_MODULE_LOG_LEVEL_DEFAULT](configs.md#config_watchdog_module_log_level_default)
> >
> > **Menu Path:** `(Top) > Beluga configurations > Beluga Debugging > Logger configurations > Beluga Service Client > Beluga Service > Serial > Serial Comms > Comms Backend > App LEDs > AT Commands > Beluga Message > BLE Application > Debug > Initiator > List Monitor > Neighbor Listing > Main Application > Power Manager > Range Extension > Ranging > Responder > Serial LEDs > Settings > SPI module > Voltage Regulator > Watchdog`

### Beluga Debug

### Beluga Threads

### Comms Module

### Neighbors

### UWB

### Watchdog

### Miscellaneous

## External Flash Configuration

## Signing Images

## Device tree
