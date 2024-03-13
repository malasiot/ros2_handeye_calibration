#include "image_widget.hpp"

#include <opencv2/opencv.hpp>

#include <QTimer>
#include <QToolButton>
#include <QCoreApplication>
#include <QMenu>
#include <QMouseEvent>
#include <QScrollBar>
#include <QApplication>
#include <QDesktopWidget>
#include <QPaintEngine>
#include <QDebug>
#include <QFileInfo>
#include <QUrl>
#include <QMimeData>
#include <QGraphicsSceneMouseEvent>

using namespace std;


QImageWidget::QImageWidget(QWidget *parent): QGraphicsView(parent)
{
    setMouseTracking(true);


    gscene = new QImageGraphicsScene(this) ;
    setScene(gscene) ;

    zc = 7 ;

    ctxMenu = NULL ;

    QToolButton *cornerWidget = new QToolButton ;
    cornerWidget->setIcon(QIcon(":/images/scroll.png")) ;
    QScrollPopup *popup = new QScrollPopup(this) ;

    setCornerWidget(cornerWidget) ;
    connect(cornerWidget, SIGNAL(pressed()), popup, SLOT(popup())) ;

    setAcceptDrops(true);

    has_image = false ;
    pixmapItem = 0 ;
}




bool QImageWidget::eventFilter(QObject *o, QEvent *e)
{

    if (e->type() == QEvent::Wheel )
    {
        if ( o == (QObject *)verticalScrollBar() ) return true ;
        else if ( o == parent() )
        {
            QCoreApplication::sendEvent(this, e) ;
            return true ;
        }
        else return false ;
    }


    return QGraphicsView::eventFilter(o, e) ;

}



void QImageGraphicsScene::mouseMoveEvent ( QGraphicsSceneMouseEvent * mouseEvent )
{

    QGraphicsScene::mouseMoveEvent(mouseEvent) ;
    view->mouseMoveEventHandler( mouseEvent ) ;
}


void QImageWidget::mouseMoveEventHandler( QGraphicsSceneMouseEvent * mouseEvent )
{

 //   if ( currentTool ) currentTool->mouseMoved(mouseEvent);

}

QString QImageWidget::sampleImage(const cv::Mat &im, const QPointF &pos)
{
    int i = pos.y() ;
    int j = pos.x() ;

    char buffer[80] ;

    if ( i > im.rows - 1 || i < 0 || j > im.cols - 1 || j < 0 ) return QString() ;

    if ( im.type() == CV_8UC1 )
    {
        sprintf(buffer, "(%d %d) \n%d", j, i, im.at<uchar>(i, j)) ;
    }
    else if ( im.type() == CV_8UC3 )
    {
        cv::Vec3b clr = im.at<cv::Vec3b>(i, j) ;
        sprintf(buffer, "(%d %d) \n%d %d %d", j, i, clr.val[0], clr.val[1], clr.val[2]) ;
    }
    else if ( im.type() == CV_16UC1 )
    {
        sprintf(buffer, "(%d %d) \n%d", j, i, (int)im.at<ushort>(i, j)) ;
    }
    else if ( im.type() == CV_32FC1 )
    {
        sprintf(buffer, "(%d %d) \n%f", j, i, im.at<float>(i, j)) ;
    }


    return QString(buffer) ;
}



void QImageGraphicsScene::mousePressEvent ( QGraphicsSceneMouseEvent * mouseEvent )
{

    QGraphicsScene::mousePressEvent(mouseEvent) ;
    view->mousePressEventHandler(mouseEvent) ;
}


void QImageWidget::mousePressEventHandler( QGraphicsSceneMouseEvent * mouseEvent )
{
    if ( mouseEvent->button() == Qt::RightButton && ctxMenu )
    {
        ctxMenu->popup(mouseEvent->screenPos()) ;
    }

    setDragMode(QGraphicsView::ScrollHandDrag) ;

   // if ( currentTool ) currentTool->mousePressed(mouseEvent);

}

void QImageGraphicsScene::mouseReleaseEvent( QGraphicsSceneMouseEvent * mouseEvent )
{


    QGraphicsScene::mouseReleaseEvent(mouseEvent) ;
    view->mouseReleaseEventHandler(mouseEvent) ;


}

void QImageWidget::mouseReleaseEventHandler( QGraphicsSceneMouseEvent * mouseEvent )
{
    setDragMode(QGraphicsView::NoDrag) ;
}

static void hsv2rgb(float h, QRgb &rgb)
{
    int i ;
    float f, p, q, t, r, g, b ;

    if ( h == 0.0 ) return ;

    // h = 360.0-h ;

    h /= 60.0 ;

    i = (int)h ;
    f = h - i ;
    p = 0  ;
    q = 1-f ;
    t = f ;

    switch (i)
    {
    case 0:
        r = 1 ;
        g = t ;
        b = p ;
        break ;
    case 1:
        r = q ;
        g = 1 ;
        b = p ;
        break ;
    case 2:
        r = p ;
        g = 1 ;
        b = t ;
        break ;
    case 3:
        r = p ;
        g = q ;
        b = 1 ;
        break ;
    case 4:
        r = t ;
        g = p ;
        b = 1 ;
        break ;
    case 5:
        r = 1 ;
        g = p ;
        b = q ;
        break ;
    }

    rgb = qRgb((int)(255.0*r), (int)(255.0*g), (int)(255.0*b)) ;
}

const int nColors = 2 << 12 ;
QRgb *QImageWidget::hsvlut ;

float QImageWidget::imgMinVal = FLT_MIN ;
float QImageWidget::imgMaxVal = FLT_MAX ;
bool QImageWidget::useColourCoding = true ;

cv::Mat QImageWidget::image() const
{
    return curImage ;
}

QGraphicsScene *QImageWidget::scene() {
    return static_cast<QGraphicsScene *>(gscene) ;
}


QPixmap *QImageWidget::imageToPixmap(const cv::Mat &img)
{
    return new QPixmap(QPixmap::fromImage(imageToQImage(img))) ;
}

QImage QImageWidget::imageToQImage(const cv::Mat &img)
{
    int w = img.cols, h = img.rows, lw = img.step[0] ;


    if ( img.type() == CV_8UC1 )
    {
        QImage image((uchar *)img.data, w, h, lw, QImage::Format_Indexed8) ;
        /*
            QVector<QRgb> colors ;
            for( int i=0 ; i<256 ; i++ ) colors.append(QColor(i, i, i).rgba()) ;

            image.setColorTable(colors) ;

        */	return image ;
    }
    else if ( img.type() == CV_8UC3 )
    {
        QImage image(img.cols, img.rows, QImage::Format_RGB32) ;

        for( int i=0 ; i<img.rows ; i++ )
        {
            uchar *dst = image.scanLine(i), *src = (uchar *)img.ptr<uchar>(i) ;

            for( int j=0 ; j<img.cols ; j++ )
            {
                uchar B = *src++ ;
                uchar G = *src++ ;
                uchar R = *src++ ;

                *(QRgb *)dst = qRgb(R, G, B) ;
                dst += 4 ;

            }
        }

        return image ;
    }
    else if ( img.type() == CV_8UC4 )
    {
        QImage image(img.cols, img.rows, QImage::Format_ARGB32) ;

        for( int i=0 ; i<img.rows ; i++ )
        {
            uchar *dst = image.scanLine(i), *src = (uchar *)img.ptr<uchar>(i) ;

            for( int j=0 ; j<img.cols ; j++ )
            {
                uchar B = *src++ ;
                uchar G = *src++ ;
                uchar R = *src++ ;
                uchar A = *src++ ;

                *(QRgb *)dst = qRgba(R, G, B, A) ;
                dst += 4 ;

            }
        }

        return image ;
    }
    else if ( img.type() == CV_16UC1 )
    {
        int nc = nColors ;

        if ( !hsvlut )
        {
            int c ;
            float h, hmax, hstep ;

            hsvlut = new QRgb [nColors] ;

            hmax = 180 ;
            hstep = hmax/nc ;

            for ( c=0, h=hstep ; c<nc ; c++, h += hstep) hsv2rgb(h, hsvlut[c]) ;
        }

        unsigned short minv, maxv ;
        int i, j ;

        minv = 0xffff ;
        maxv = 0 ;

        uchar *ppl = img.data ;
        unsigned short *pp = (unsigned short *)ppl ;

        for ( i=0 ; i<h ; i++, ppl += lw )
            for ( j=0, pp = (unsigned short *)ppl ; j<w ; j++, pp++ )
            {
                if ( *pp == 0 ) continue ;
                maxv = qMax(*pp, maxv) ;
                minv = qMin(*pp, minv) ;
            }

        QImage image(w, h, QImage::Format_RGB32) ;

        for( i=0 ; i<h ; i++ )
        {
            uchar *dst = image.scanLine(i) ;
            unsigned short *src = (unsigned short *)img.ptr<ushort>(i) ;

            for( j=0 ; j<w ; j++ )
            {
                unsigned short val = *src++ ;

                if ( val == 0 )
                {
                    *(QRgb *)dst = Qt::black ;
                    dst += 3 ;
                    *dst++ = 255 ;

                    continue ;
                }
                else val = (nc-1)*float((val - minv)/float(maxv - minv)) ;

                const QRgb &clr = hsvlut[val] ;

                *(QRgb *)dst = clr ;
                dst += 3 ;
                *dst++ = 255 ;
            }
        }

        return image ;

    }


    return QImage() ;
}


cv::Mat QImageWidget::QImageToImage(const QImage &im)
{
    int w = im.width(), h = im.height(), bpl = im.bytesPerLine() ;
    const uchar *srcData = im.bits() ;

    if ( im.format() == QImage::Format_Indexed8 && im.allGray() )
    {
        cv::Mat_<uchar> res(h, w) ;

        const uchar *src = srcData ;

        for(int i=0 ; i<h ; i++, src += bpl )
            memcpy(res.ptr<uchar>(i), src, w) ;

        return res ;
    }
    else if ( im.format() == QImage::Format_Indexed8 )
    {
        QVector<QRgb> table = im.colorTable() ;

        cv::Mat_<cv::Vec3b> res(h, w) ;

        int i, j ;

        const uchar *src = srcData ;
        uchar *dst = (uchar *)res.data ;

        for(i=0 ; i<h ; i++, src += bpl, dst += res.step[0] )
        {
            const uchar *sp = src ;
            uchar *dp = dst ;

            for(j=0 ; j<w ; j++ )
            {
                QRgb val(table[*sp++]) ;
                *dp++ = qBlue(val) ;
                *dp++ = qGreen(val) ;
                *dp++ = qRed(val) ;
            }
        }

        return res ;
    }
    else if ( im.format() == QImage::Format_RGB32 )
    {
        cv::Mat_<cv::Vec3b> res(h, w) ;

        int i, j ;

        const QRgb *src = (const QRgb *)srcData ;
        uchar *dst = (uchar *)res.data ;

        for(i=0 ; i<h ; i++, src += w, dst += res.step[0] )
        {
            const QRgb *sp = src ;
            uchar *dp = dst ;

            for(j=0 ; j<w ; j++ )
            {
                QRgb val = *sp++ ;
                *dp++ = qBlue(val) ;
                *dp++ = qGreen(val) ;
                *dp++ = qRed(val) ;
            }
        }
        return res ;
    }
    else if ( im.format() == QImage::Format_ARGB32 ) {
        cv::Mat res(h, w, CV_8UC4) ;

        int i, j ;

        const QRgb *src = (const QRgb *)srcData ;
        uchar *dst = (uchar *)res.data ;

        for(i=0 ; i<h ; i++, src += w, dst += res.step[0] )
        {
            const QRgb *sp = src ;
            uchar *dp = dst ;

            for(j=0 ; j<w ; j++ )
            {
                QRgb val = *sp++ ;

                *dp++ = qBlue(val) ;
                *dp++ = qGreen(val) ;
                *dp++ = qRed(val) ;
                *dp++ = qAlpha(val) ;
            }
        }


        return res ;


    }

    return cv::Mat() ;
}


void QImageWidget::setImage(const cv::Mat &im)
{
    if ( pixmapItem ) delete pixmapItem ;
    QPixmap *p = imageToPixmap(im) ;

    if ( !p ) return  ;

    curImage = im ;
    pixmap = p ;

    has_image = true ;

    pixmapItem = (QGraphicsPixmapItem *)gscene->addPixmap(*p) ;
    pixmapItem->setZValue(-1.0) ;

    setSceneRect(QRectF(0, 0, pixmap->width(), pixmap->height())) ;

    verticalScrollBar()->installEventFilter(this);

    update() ;
}

void QImageWidget::setImage(const QImage &qim)
{

    if ( pixmapItem ) delete pixmapItem ;
    QPixmap *p = new QPixmap(QPixmap::fromImage(qim)) ;

    if ( !p ) return  ;

    curImage = QImageToImage(qim) ;
    pixmap = p ;

    has_image = true ;

    pixmapItem = (QGraphicsPixmapItem *)gscene->addPixmap(*p) ;
    pixmapItem->setZValue(-1.0) ;

    setSceneRect(QRectF(0, 0, pixmap->width(), pixmap->height())) ;

    verticalScrollBar()->installEventFilter(this);
}

void QImageWidget::addItem(QGraphicsItem *item)
{
    gscene->addItem(item) ;
}


static float zoomFactors[] = { 0.02, 0.05, 0.1, 0.25, 0.33, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0,
                               9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0
                             } ;

void QImageWidget::setZoom( int cc )
{
    if ( zc == cc ) return ;

    zc = cc;

    float zoomFactor = zoomFactors[cc] ;

    QTransform tr = transform() ;

    tr.setMatrix(zoomFactor, tr.m12(), tr.m13(), tr.m21(), zoomFactor, tr.m23(),
                 tr.m31(), tr.m32(), tr.m33()) ;

    setTransform(tr) ;

    emit zoomChanged(zc) ;
}

float QImageWidget::getZoomFactor() const {
    return zoomFactors[zc] ;
}

void QImageWidget::wheelEvent( QWheelEvent *event )
{
    int zz = zc  ;
    if ( event->delta() < 0 ) zz -- ;
    else zz++ ;

    if ( zz < 0 ) zz = 0 ;

    if( zz >= sizeof(zoomFactors)/sizeof(float)) zz = sizeof(zoomFactors)/sizeof(float)-1 ;

    setZoom( zz );
}

void QImageWidget::zoomRel( int delta )
{
    int zz = zc + delta   ;

    if ( zz < 0 ) zz = 0 ;

    if( zz >= sizeof(zoomFactors)/sizeof(float)) zz = sizeof(zoomFactors)/sizeof(float)-1 ;

    setZoom( zz );
}

void QImageWidget::zoomToPoint(const QPointF &p, int delta)
{
    int zz = zc + delta   ;

    if ( zz < 0 ) zz = 0 ;

    if( zz >= sizeof(zoomFactors)/sizeof(float)) zz = sizeof(zoomFactors)/sizeof(float)-1 ;

    if ( zc == zz ) return ;

    zc = zz;

    resetTransform() ;

    scale(zoomFactors[zc], zoomFactors[zc]) ;
    centerOn(p) ;

    emit zoomChanged(zc) ;
}

void QImageWidget::zoomToRect(const QRectF &rect)
{
    QRect srect = rect.toRect() ; //view->mapFromScene(rect).boundingRect() ;

    QSize sz = viewport()->size() ;
    QRect vpRect(0, 0, sz.width(), sz.height()) ;

    int cc = 0;
    for( cc = 0 ; cc < sizeof(zoomFactors)/sizeof(float) ; cc++ )
    {

        float zoomFactor = zoomFactors[cc] ;

        QTransform tr ;
        tr.translate(vpRect.center().x(), vpRect.center().y()) ;
        tr.scale(zoomFactor, zoomFactor) ;
        tr.translate(-rect.center().x(), -rect.center().y()) ;

        QRect zoomRect = tr.mapRect(rect).toRect() ;

        if ( zoomRect.isValid() && !vpRect.contains(zoomRect) ) break ;
    }

    cc = qMin(cc-1, (int)(sizeof(zoomFactors)/sizeof(float) - 1) ) ;
    cc = qMax(cc, 0) ;
    resetTransform() ;

    centerOn(rect.center()) ;
    scale(zoomFactors[cc], zoomFactors[cc]) ;

    if ( zc == cc ) return ;

    zc = cc;

    emit zoomChanged(zc) ;
}

void QImageWidget::setCanvasSize(int sw, int sh)
{
    setSceneRect(0, 0, sw, sh) ;
}

QScrollPopup::QScrollPopup(QImageWidget *p): QFrame(p,  Qt::Popup )
{
    popupParent = p ;
    setFrameStyle( QFrame::Box | QFrame::Raised);

    setMouseTracking( false );
    pix = NULL ;
}


void QScrollPopup::mouseMoveEvent( QMouseEvent * e) {
    QPoint nc = e->globalPos() ;

    int dx = nc.x() - pc.x() ;
    int dy = nc.y() - pc.y() ;

    px += dx ;
    py += dy ;

    if ( px + pww >= pvw )
    {
        px = pvw - pww ;
    }

    if ( py + pwh >= pvh )
    {
        py = pvh - pwh ;
    }

    if ( px < 0 ) px = 0 ;

    if ( py < 0 ) py = 0 ;

    QScrollBar *hsb = popupParent->horizontalScrollBar() ;
    QScrollBar *vsb = popupParent->verticalScrollBar() ;

    hsb->setValue(px * hsb->maximum()/(float)(pvw-pww)) ;
    vsb->setValue(py * vsb->maximum()/(float)(pvh-pwh)) ;

    pc = nc ;

    update() ;

}

void QScrollPopup::mouseReleaseEvent( QMouseEvent * e)
{
    //   if  (rect().contains( e->pos() ) || moves > 0)
    delete pix ;
    close();
}

void QScrollPopup::closeEvent( QCloseEvent *e ) {
    e->accept();
}

#define SCROLL_POPUP_MAXSIZE 120
#define SCROLL_POPUP_MARGIN 3

void QScrollPopup::paintEvent(QPaintEvent *e)
{
    QPainter painter ;
    painter.begin(this) ;

    painter.drawPixmap(QRect(SCROLL_POPUP_MARGIN, SCROLL_POPUP_MARGIN, pvw, pvh), *pix) ;

    painter.setPen("white") ;
    painter.drawRect(QRect(SCROLL_POPUP_MARGIN + px, SCROLL_POPUP_MARGIN + py, pww, pwh)) ;
    painter.fillRect(QRect(SCROLL_POPUP_MARGIN + px, SCROLL_POPUP_MARGIN + py, pww, pwh), QBrush(QColor(255, 255, 255, 30))) ;
}

void QScrollPopup::popup() {
    int dw = QApplication::desktop()->width() ;
    int dh = QApplication::desktop()->height() ;

    QTransform tr = popupParent->viewportTransform() ;
    origViewRect = tr.mapRect(popupParent->sceneRect()) ;
    QRect geom = popupParent->geometry() ;

    vw = origViewRect.width() ;
    vh = origViewRect.height() ;
    winw = geom.width() ;
    winh = geom.height() ;
    vx = origViewRect.left() ;
    vy = origViewRect.top() ;

    if ( vw > vh )
    {
        pvw = SCROLL_POPUP_MAXSIZE ;
        float scale = vh / (float) vw ;
        pvh = pvw * scale ;
        px = -vx * pvw / (float)vw ;
        py = -vy * pvw / (float)vw ;
    }
    else
    {
        pvh = SCROLL_POPUP_MAXSIZE ;
        float scale = vw / (float) vh ;
        pvw = pvh * scale ;
        px = -vx * pvh / (float)vh ;
        py = -vy * pvh / (float)vh ;
    }

    pwh = pvh * winh / (float) vh ;
    pww = pvw * winw / (float) vw ;

    int ox = px + pww/2, oy = py + pwh/2 ;

    QWidget *cornerWidget = popupParent->cornerWidget() ;
    pc = popupParent->mapToGlobal(cornerWidget->geometry().center()) ;
    QPoint pc2 = pc + QPoint(-ox, -oy);

    if ( pc2.x() + pvw >= dw )
    {
        pc2.rx() = dw - pvw-1 ;
    }

    if ( pc2.y() + pvh >= dh )
    {
        pc2.ry() = dh - pvh -1 ;
    }

    pc = pc2 + QPoint(ox, oy) ;


    pix = new QPixmap(pvw, pvh) ;
    QPainter painter ;
    painter.begin(pix) ;
    popupParent->scene()->render(&painter) ;

    resize(pvw+SCROLL_POPUP_MARGIN+SCROLL_POPUP_MARGIN, pvh+SCROLL_POPUP_MARGIN+SCROLL_POPUP_MARGIN) ;
    move( pc2) ;
    QCursor::setPos(pc) ;
    show();
}


