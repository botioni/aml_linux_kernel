#ifndef __CAPTS_H__
#define __CAPTS_H__

#define EVENT_MAX   5


struct ts_event {
    short x;
    short y;
    short z;
    short w;
    unsigned char id;
    unsigned char state;
};


struct ts_info {
    unsigned short xmin;
    unsigned short xmax;
    unsigned short ymin;
    unsigned short ymax;
    unsigned short zmin;
    unsigned short zmax;
    unsigned short wmin;
    unsigned short wmax;

    unsigned short swap_xy:1;
    unsigned short x_pol:1;
    unsigned short y_pol:1;
};


struct ts_platform_data {
    int irq;
    int (*init_irq)(void);
    int (*get_irq_level)(void);
    struct ts_info info;
};

struct ts_chip {
    char *name;
    char *version;
    int (*reset)(struct device *dev);
    int (*calibration)(struct device *dev);
    int (*get_event)(struct device *dev, struct ts_event *event);
};

extern int capts_probe(struct device *dev, struct ts_chip *chip);
extern int capts_remove(struct device *dev);

#endif