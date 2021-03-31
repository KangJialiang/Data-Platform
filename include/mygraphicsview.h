#ifndef MYGRAPHICSVIEW_H
#define MYGRAPHICSVIEW_H

#include <QObject>
#include <QGraphicsView>

class MyGraphicsView : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit MyGraphicsView(QObject *parent = nullptr);
    void _do_zoom( const QRectF & data_extent );
    void calc_full_scale();


signals:

public slots:
};

#endif // MYGRAPHICSVIEW_H
