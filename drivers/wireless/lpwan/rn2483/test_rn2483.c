#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>

#include <nuttx/wireless/lpwan/rn2483.h>
#include <nuttx/wireless/ioctl.h>
#include <nuttx/config.h>

#define SET "set"
#define GET "get"

void check_err(int err, char *op, char* param){
    if (err == 0)
    {
        syslog(LOG_INFO, "PASS: %s %s\n", op, param);
    }
    else if (err < 0)
    {
        syslog(LOG_INFO, "FAIL(error %d): %s %s\n", err, op, param);
    }
    else
    {
        syslog(LOG_INFO, "UNDEFINED(error %d): %s %s\n", err, op, param);
    }

    return;
}

void check_set_and_get_values(void* set_val, void* get_val)
{
    if (*set_val == *get_val)
    {
        syslog(LOG_INFO, "PASS: values match:\n");
    }
    else
    {
        syslog(LOG_INFO, "FAIL: values don't match:\n");
    }
}

int main(int argc, char *argv[])
{
    syslog(LOG_INFO, "ENTER TEST\n");
    const char *filename = "rn2483";
    int fd = open(filename, O_WRONLY, 0644);
    int err;

    /*   Test set mod   */
    syslog(LOG_INFO, "TEST set mod\n");
    enum rn2483_mod_e set_mod = RN2483_MOD_LORA;
    err = ioctl(fd, WLIOC_SETRADIOMOD, &set_mod);
    check_err(err, SET, "mod");

    /*   Test get mod   */
    syslog(LOG_INFO, "TEST get mod\n");
    enum rn2483_mod_e get_mod;
    err = ioctl(fd, WLIOC_GETRADIOMOD, &get_mod);
    check_err(err, GET, "mod");
    check_set_and_get_values(&set_mod, &get_mod);

    /*   Test set cr   */
    syslog(LOG_INFO, "TEST set cr\n");
    enum rn2483_cr_e set_cr = RN2483_CR_4_7;
    err = ioctl(fd, WLIOC_SETRADIOCR, &set_cr);
    check_err(err, SET, "cr");

    /*   Test get cr   */
    syslog(LOG_INFO, "TEST get cr\n");
    enum rn2483_cr_e get_cr;
    err = ioctl(fd, WLIOC_GETRADIOCR, &get_cr);
    check_err(err, GET, "cr");
    check_set_and_get_values(&set_cr, &get_cr);

    /*   Test set freq   */
    syslog(LOG_INFO, "TEST set freq\n");
    uint32_t set_freq = 433050000;
    err = ioctl(fd, WLIOC_SETRADIOFREQ, &set_freq);
    check_err(err, SET, "freq");

    /*   Test get freq   */
    syslog(LOG_INFO, "TEST get freq\n");
    uint32_t get_freq;
    err = ioctl(fd, WLIOC_GETRADIOFREQ, &get_freq);
    check_err(err, GET, "freq");
    check_set_and_get_values(&set_freq, &get_freq);


    /*   Test set pwr   */
    syslog(LOG_INFO, "TEST set pwr\n");
    int8_t set_pwr = 15;
    err = ioctl(fd, WLIOC_SETRADIOPWR, &set_pwr);
    check_err(err, SET, "pwr");

    /*   Test get pwr   */
    syslog(LOG_INFO, "TEST get pwr\n");
    int8_t get_pwr;
    err = ioctl(fd, WLIOC_GETRADIOPWR, &get_pwr);
    check_err(err, GET, "pwr");
    check_set_and_get_values(&set_pwr, &get_pwr);


    /*   Test set sf   */
    syslog(LOG_INFO, "TEST set sf\n");
    int8_t set_sf = 15;
    err = ioctl(fd, WLIOC_SETRADIOSF, &set_sf);
    check_err(err, SET, "sf");

    /*   Test get sf   */
    syslog(LOG_INFO, "TEST get sf\n");
    int8_t get_sf;
    err = ioctl(fd, WLIOC_GETRADIOSF, &get_sf);
    check_err(err, GET, "sf");
    check_set_and_get_values(&set_sf, &get_sf);


    /*   Test set prlen   */
    syslog(LOG_INFO, "TEST set prlen\n");
    uint16_t set_prlen = 15;
    err = ioctl(fd, WLIOC_SETRADIOPRLEN, &set_prlen);
    check_err(err, SET, "prlen");

    /*   Test get prlen   */
    syslog(LOG_INFO, "TEST get prlen\n");
    uint16_t get_prlen;
    err = ioctl(fd, WLIOC_GETRADIOPRLEN, &get_prlen);
    check_err(err, GET, "prlen");
    check_set_and_get_values(&set_prlen, &get_prlen);


    /*   Test set crc   */
    syslog(LOG_INFO, "TEST set crc\n");
    bool set_crc = 1;
    err = ioctl(fd, WLIOC_SETRADIOCRC, &set_crc);
    check_err(err, SET, "crc");

    /*   Test get crc   */
    syslog(LOG_INFO, "TEST get crc\n");
    bool get_crc;
    err = ioctl(fd, WLIOC_GETRADIOCRC, &get_crc);
    check_err(err, GET, "crc");
    check_set_and_get_values(&set_crc, &get_crc);


    /*   Test set iqi   */
    syslog(LOG_INFO, "TEST set iqi\n");
    bool set_iqi = 1;
    err = ioctl(fd, WLIOC_SETRADIOIQI, &set_iqi);
    check_err(err, SET, "iqi");

    /*   Test get iqi   */
    syslog(LOG_INFO, "TEST get iqi\n");
    bool get_iqi;
    err = ioctl(fd, WLIOC_GETRADIOIQI, &get_iqi);
    check_err(err, GET, "iqi");
    check_set_and_get_values(&set_iqi, &get_iqi);


    /*   Test set sync   */
    syslog(LOG_INFO, "TEST set sync\n");
    uint8_t set_sync = 0x43;
    err = ioctl(fd, WLIOC_SETRADIOSYNC, &set_sync);
    check_err(err, SET, "sync");

    /*   Test get sync   */
    syslog(LOG_INFO, "TEST get sync\n");
    bool get_sync;
    err = ioctl(fd, WLIOC_GETRADIOSYNC, &get_sync);
    check_err(err, GET, "sync");
    check_set_and_get_values(&set_sync, &get_sync);

    /*   Test set bw   */
    syslog(LOG_INFO, "TEST set bw\n");
    uint16_t set_bw = 500;
    err = ioctl(fd, WLIOC_SETRADIOBW, &set_bw);
    check_err(err, SET, "bw");

    /*   Test get bw   */
    syslog(LOG_INFO, "TEST get bw\n");
    bool get_bw;
    err = ioctl(fd, WLIOC_GETRADIOBW, &get_bw);
    check_err(err, GET, "bw");
    check_set_and_get_values(&set_bw, &get_bw);
}