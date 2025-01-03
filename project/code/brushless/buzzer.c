#include "buzzer.h"

static uint32_t buzzer_period = 0; // 按键的扫描周期(ms)
bool buzzer_on_flag = true;

buzz_t buzz_left = {0};
buzz_t buzz_right = {0};
buzz_t buzz_middle = {0};

void buzzer_init(int16_t t)
{
    buzzer_period = t;
}

// 放中断
void buzz_exec(buzz_t *__buzz_)
{
    if (__buzz_->buzzer_number > 0)
    {
        if (__buzz_->buzzer_list[__buzz_->buzzer_head].time > 0)
        {
            if (__buzz_->buzzer_list[__buzz_->buzzer_head].freq == 0)
                __buzz_->buzzer_freq = 0;
            else
                __buzz_->buzzer_freq = __buzz_->buzzer_list[__buzz_->buzzer_head].freq;
            __buzz_->buzzer_list[__buzz_->buzzer_head].time -= buzzer_period;
        }
        else
        {
            __buzz_->buzzer_head = (__buzz_->buzzer_head + 1) % BUZZER_QUEUE_SIZE;
            __buzz_->buzzer_number--;
        }
    }
    else if (buzzer_on_flag == true)
    {
        buzzer_on_flag = false;
        __buzz_->buzzer_freq = 0;
    }
}

void buzz_keep_ms(int16_t t, uint16_t bfreq, buzz_t *__buzz_)
{
    buzzer_on_flag = true;
    __buzz_->buzzer_list[__buzz_->buzzer_tail].time = t;
    __buzz_->buzzer_list[__buzz_->buzzer_tail].freq = bfreq;
    __buzz_->buzzer_tail = (__buzz_->buzzer_tail + 1) % BUZZER_QUEUE_SIZE;
    __buzz_->buzzer_number++;
}

void buzz_ease_ms(int16_t t, uint16_t bfreq1, uint16_t bfreq2, buzz_t *__buzz_)
{
    // if (t >= BUZZER_EASE_DIVISOR)
    //     zf_log(0, "t必须大于缓动的最小细分时间");
    buzzer_on_flag = true;
    for (int16_t i = 0; i < BUZZER_EASE_DIVISOR - 1; i++)
    {
        __buzz_->buzzer_list[__buzz_->buzzer_tail].time = t / BUZZER_EASE_DIVISOR;
        __buzz_->buzzer_list[__buzz_->buzzer_tail].freq = bfreq1 + (bfreq2 - bfreq1) * i / BUZZER_EASE_DIVISOR;
        __buzz_->buzzer_tail = (++__buzz_->buzzer_tail) % BUZZER_QUEUE_SIZE;
    }
    __buzz_->buzzer_number += BUZZER_EASE_DIVISOR - 1;
}

const note melody[] = {
    {NOTE_G4, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_C5, 1. / 4.},
    {NOTE_G4, 2. / 4.},
    {NOTE_C5, 1. / 8.},
    {NOTE_D5, 1. / 8.},
    {NOTE_E5, 1. / 8.},
    {NOTE_D5, 1. / 8.},
    {NOTE_C5, 1. / 8.},
    {NOTE_D5, 1. / 8.},

    {NOTE_G4, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_C5, 1. / 4.},
    {NOTE_G4, 2. / 4.},
    {NOTE_C5, 1. / 8.},
    {NOTE_D5, 1. / 8.},
    {NOTE_E5, 1. / 8.},
    {NOTE_D5, 1. / 8.},
    {NOTE_C5, 1. / 8.},
    {NOTE_D5, 1. / 8.},

    {NOTE_G4, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_C5, 1. / 4.},
    {NOTE_G4, 2. / 4.},
    {NOTE_C5, 1. / 8.},
    {NOTE_D5, 1. / 8.},
    {NOTE_E5, 1. / 8.},
    {NOTE_D5, 1. / 8.},
    {NOTE_C5, 1. / 8.},
    {NOTE_D5, 1. / 8.},

    {NOTE_G4, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_C5, 1. / 4.},
    {NOTE_G4, 2. / 4.},

    {NOTE_C5, 1. / 4.},
    {NOTE_E5, 1. / 4.},
    {NOTE_G5, 1. / 4.},

    {NOTE_A5, 2. / 4.},
    {NOTE_G5, 1. / 4.},
    {NOTE_G5, 2. / 4.},
    {NOTE_A5, 2. / 4.},
    {NOTE_C5, 1. / 4.},

    {NOTE_D5, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_C5, 1. / 4.},
    {NOTE_E5, 4. / 4.},
    {NOTE_E5, 1. / 8.},
    {NOTE_G5, 1. / 8.},

    {NOTE_A5, 1. / 4.},
    {NOTE_B5, 1. / 4.},
    {NOTE_A5, 1. / 4.},
    {NOTE_G5, 1. / 4.},
    {NOTE_E5, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_C5, 1. / 4.},

    {NOTE_D5, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_E5, 1. / 4.},
    {NOTE_C5, 4. / 4.}, //
    {NOTE_E5, 1. / 4.},
    {NOTE_G5, 1. / 4.},

    {NOTE_A5, 2. / 4.},
    {NOTE_G5, 1. / 4.},
    {NOTE_G5, 2. / 4.},
    {NOTE_A5, 2. / 4.},
    {NOTE_C5, 1. / 4.},

    {NOTE_D5, 1. / 4.},
    {NOTE_D5, 1. / 4.},
    {NOTE_C5, 1. / 4.},
    {NOTE_E5, 4. / 4.},
    {NOTE_E5, 1. / 8.},
    {NOTE_G5, 1. / 8.},

    {NOTE_A5, 1. / 4.},
    {NOTE_C6, 1. / 4.},
    {NOTE_D6, 1. / 4.},
    {NOTE_E6, 1. / 4.},
    {NOTE_D6, 1. / 4.},
    {NOTE_C6, 1. / 4.},
    {NOTE_A5, 1. / 4.},
    {NOTE_C6, 1. / 4.},

    {NOTE_D6, 1. / 4.},
    {NOTE_D6, 1. / 4.},
    {NOTE_C6, 1. / 4.},
    {NOTE_C6, 3. / 4.},
    {NOTE_A5, 1. / 8.},
    {NOTE_C6, 1. / 8.},

    {NOTE_D6, 1. / 4.},
    {NOTE_D6, 1. / 4.},
    {NOTE_C6, 1. / 4.},
    {NOTE_C6, 4. / 4.},
};


void play_music()
{
    int16 bpm = 360;
    int16 notes = 92;
    int16 wholenote = 60000 * 4 / bpm;
    for (int16 thisNote = 0; thisNote < notes; thisNote = thisNote++)
    {
        int16 noteDuration = wholenote * melody[thisNote].divider;

        buzz_keep_ms(noteDuration * 0.9, melody[thisNote].freq * 2, &buzz_left);
        buzz_keep_ms(noteDuration * 0.1, 0, &buzz_left);

        buzz_keep_ms(noteDuration * 0.9, melody[thisNote].freq * 2, &buzz_right);
        buzz_keep_ms(noteDuration * 0.1, 0, &buzz_right);
    }
}

void play_cxk()
{
    // 40403
    // buzz_left.buzzer_freq = 10000;
    // buzz_right.buzzer_freq = 10000;

    // if (i < 40402)
    //     i++;
}