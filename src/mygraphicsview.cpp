#include "mygraphicsview.h"
#include <QRect>
#include <QMatrix>
#include <QGraphicsView>
#include <QGraphicsAnchor>

MyGraphicsView::MyGraphicsView(QGraphicsView *parent) : QObject(parent)
{

}
void MyGraphicsView::_do_zoom(QRectF &data_extent )
{
    QRect abc = viewport()->geometry();
    double scale_x = abc.width() /data_extent.width() ;
    double scale_y = abc.height() / data_extent.height();
    if( scale_x > scale_y ) scale_x = scale_y;

    QMatrix matrix( scale_x, 0, 0, scale_x,
                    ( abc.center().x() - data_extent.center().x()*scale  )  ,
                    ( abc.center().y() - data_extent.center().y()*scale  )) ;
   //此处直接改变了 坐标系统的 偏移， 即直接偏移映射， 无需改变widget的逻辑窗口位置（centerOn）//

setMatrix( matrix) ;

}
double MyGraphicsView::calc_full_scale()
{
    double dx = viewport()->geometry().width()/scene()->width();
    double dy = viewport()->geometry().height()/scene()->height();
    // return min( dx, dy) ;
    return dx < dy ? dx : dy ;
}
void MyGraphicsView::_do_zoom( double scale_factor, const QPoint & anchor )
{
    assert( scale_factor > 0 );


/*	double scale = transform().m11(); //( assert( f.m11() = f.m22());
    double full_scale = calc_full_scale();
    double dest_scale = scale * scale_factor ;
*/
    // 根据需要，限制缩放极限值
    // if( scale_factor > 1.0 && ( dest_scale > 2.0 || dest_scale > full_scale * 10.0 ) ) return ;
    // if( scale_factor < 1.0 && dest_scale < full_scale * .075 ) return ;

    // device_point--2--logic_point--2--data_point
    QPointF data_anchor = mapToScene( anchor) ;

    QGraphicsView::scale( scale_factor, scale_factor );

    // distance = (device_point -- widget_center)
    QPoint widget_center = viewport()->geometry().center();
    QPoint dis = widget_center - anchor ;

    QPointF new_data_center = QPointF( data_anchor.x() + dis.x() / dest_scale , data_anchor.y() + dis.y() / dest_scale );
    centerOn( new_data_center);

}
