#pragma once

#include <QGraphicsView>
#include <QGraphicsItem>

#include <opencv2/opencv.hpp>

class QImageWidget : public QGraphicsView
{
    Q_OBJECT

public:

    QImageWidget(QWidget *parent);

    // the image currently displayed
    cv::Mat image() const ;

    // the underlying graphics scene
    QGraphicsScene *scene();



    // set canvas size
    void setCanvasSize(int sw, int sh) ;

    // get the selection of the rectangular rubber band
    QRect getRectSelection() const ;

    // get the selection of the polygon tool
    QPolygon getPolySelection() const ;

    // change the current selection of the polygon tool
    void setPolySelection(QPolygon &poly) ;

    bool hasImage() const {
        return has_image ;
    }

    // use this to add annotations over the image
    void addItem(QGraphicsItem *item) ;

    // conversions

    static QImage imageToQImage(const cv::Mat &im) ;
    static cv::Mat QImageToImage(const QImage &px) ;

    static QPixmap *imageToPixmap(const cv::Mat &im) ;
    static cv::Mat pixmapToImage(const QPixmap &px) ;

    QString userFriendlyCurrentFile();

    QString sampleImage(const cv::Mat &im, const QPointF &pos) ;

//    QImageTool *setTool(QImageTool *tool) ;

 //   QImageTool *getCurrentTool() const {
  //      return currentTool ;
  //  }

    float getZoomFactor() const ;

    int getZoom() const {
        return zc ;
    }

public:

    static bool useColourCoding ;
    static float imgMinVal ;
    static float imgMaxVal ;

protected:


    cv::Mat curImage ;
    QPixmap *pixmap ;
    QList<QGraphicsItem *> overlays ;

    QGraphicsPixmapItem *pixmapItem ;

//    QMap<QByteArray, QImageTool *> tools ;
//    QImageTool *currentTool ;



public slots:

    void setZoom( int );
    void zoomRel( int ) ;

    void zoomToRect(const QRectF &rect) ;
    void zoomToPoint(const QPointF &pt, int delta) ;

    // set the image
    void setImage(const cv::Mat &im) ;
    void setImage(const QImage &qim) ;
private:

    friend class QImageGraphicsScene ;
    friend class QImageView ;

    void wheelEvent( QWheelEvent* );

    bool eventFilter(QObject *o, QEvent *e) ;


    void mousePressEventHandler ( QGraphicsSceneMouseEvent * mouseEvent ) ;
    void mouseMoveEventHandler ( QGraphicsSceneMouseEvent * mouseEvent ) ;
    void mouseReleaseEventHandler( QGraphicsSceneMouseEvent * mouseEvent ) ;

    class QImageGraphicsScene *gscene ;
    QMenu *ctxMenu ;

    int zc ;
    bool has_image ;

private:

    static QRgb *hsvlut ;

signals:

    void zoomChanged(int) ;
};

class  QImageGraphicsScene: public QGraphicsScene
{
    Q_OBJECT

public:

    QImageGraphicsScene(QImageWidget *v): QGraphicsScene() {
        view = v ;
    }

private:

    void mousePressEvent ( QGraphicsSceneMouseEvent * mouseEvent ) ;
    void mouseMoveEvent ( QGraphicsSceneMouseEvent * mouseEvent ) ;
    void mouseReleaseEvent( QGraphicsSceneMouseEvent * mouseEvent ) ;

    QImageWidget *view ;
} ;



class QScrollPopup : public QFrame
{
    Q_OBJECT

public:

    QScrollPopup(QImageWidget * parent);

public slots:

    void popup( );

protected:

    virtual void mouseMoveEvent( QMouseEvent * );
    virtual void mouseReleaseEvent( QMouseEvent * );
    virtual void closeEvent( QCloseEvent * );
    virtual void paintEvent(QPaintEvent *) ;

    QImageWidget* popupParent;
    QRectF origViewRect ;
    QPoint pc ;
    int vw, vh, winw, winh ;
    int vx, vy ;
    int pvw, pvh, pww, pwh, px, py ;
    QPixmap *pix ;

};



Q_DECLARE_METATYPE(cv::Mat)

