

void escint_init() {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 475;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    pwm_config.frequency = 475;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_MCPWM0A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_MCPWM0B);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, GPIO_MCPWM1A);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, GPIO_MCPWM1B);
}


float meas_battery() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);
    float voltage = 0.000268 * (float)adc1_get_raw(ADC1_CHANNEL_0);
    return ( 1.00 * voltage );
}


