#include "buzzer.h"

uint16_t foc_ud_freq;
static uint32_t buzzer_period = 0; // 按键的扫描周期(ms)
bool buzzer_on_flag = true;

buzz_t buzzer_list[BUZZER_QUEUE_SIZE];
uint32_t buzzer_number = 0;
uint32_t buzzer_head = 0;
uint32_t buzzer_tail = 0;

void buzzer_init(int16_t t)
{
    foc_ud_freq = 0;
    buzzer_period = t;
}

// 放中断
void buzz_exec()
{
    if (buzzer_number > 0)
    {
        if (buzzer_list[buzzer_head].time > 0)
        {
            if (buzzer_list[buzzer_head].freq == 0)
                foc_ud_freq = 0;
            else
                foc_ud_freq = buzzer_list[buzzer_head].freq;
            buzzer_list[buzzer_head].time -= buzzer_period;
        }
        else
        {
            buzzer_head = (buzzer_head + 1) % BUZZER_QUEUE_SIZE;
            buzzer_number--;
        }
    }
    else if (buzzer_on_flag == true)
    {
        buzzer_on_flag = false;
        foc_ud_freq = 0;
    }
}

void buzz_keep_ms(int16_t t, uint16_t bfreq)
{
    buzzer_on_flag = true;
    buzzer_list[buzzer_tail].time = t;
    buzzer_list[buzzer_tail].freq = bfreq;
    buzzer_tail = (buzzer_tail + 1) % BUZZER_QUEUE_SIZE;
    buzzer_number++;
}

void buzz_ease_ms(int16_t t, uint16_t bfreq1, uint16_t bfreq2)
{
    // if (t >= BUZZER_EASE_DIVISOR)
    //     zf_log(0, "t必须大于缓动的最小细分时间");
    buzzer_on_flag = true;
    for (int16_t i = 0; i < BUZZER_EASE_DIVISOR - 1; i++)
    {
        buzzer_list[buzzer_tail].time = t / BUZZER_EASE_DIVISOR;
        buzzer_list[buzzer_tail].freq = bfreq1 + (bfreq2 - bfreq1) * i / BUZZER_EASE_DIVISOR;
        buzzer_tail = (++buzzer_tail) % BUZZER_QUEUE_SIZE;
    }
    buzzer_number += BUZZER_EASE_DIVISOR - 1;
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
    int16 bpm = 180;
    int16 notes = 92;
    int16 wholenote = 60000 * 4 / bpm;
    for (int16 thisNote = 0; thisNote < notes; thisNote = thisNote++)
    {
        int16 noteDuration = wholenote * melody[thisNote].divider;

        buzz_keep_ms(noteDuration * 0.9, melody[thisNote].freq);
        buzz_keep_ms(noteDuration * 0.1, 0);
    }
}