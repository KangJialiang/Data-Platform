#ifndef VISUAL_TRAJ_H
#define VISUAL_TRAJ_H

#include <QWidget>
#include <QTimer>
#include <QList>

class visual_traj : public QWidget
{
    Q_OBJECT
protected:
    void paintEvent(QPaintEvent *);
public:
    int m_count;
    float x;
    QTimer* m_Timer;
    QList<float> xList;
    QList<float> yList;

    Widget(QWidget *parent = 0);
    ~Widget();

public slots:
    void creatData();

};

#endif // VISUAL_TRAJ_H
