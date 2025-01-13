#ifndef LABEL_H
#define LABEL_H


struct label_position
{
    struct label_position   *next;
    short                   position;
};
typedef struct label_position LabelPosition;

#endif