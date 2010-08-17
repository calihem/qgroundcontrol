#include "Linecharts.h"

Linecharts::Linecharts(QWidget *parent) :
        QStackedWidget(parent),
        plots(),
        active(true)
{
  this->setVisible(false);
}


void Linecharts::setActive(bool active)
{
    this->active = active;
    QWidget* prevWidget = currentWidget();
    if (prevWidget)
    {
        LinechartWidget* chart = dynamic_cast<LinechartWidget*>(prevWidget);
        if (chart)
        {
            chart->setActive(active);
        }
    }
}


void Linecharts::selectSystem(int systemid)
{
    QWidget* prevWidget = currentWidget();
    if (prevWidget)
    {
        LinechartWidget* chart = dynamic_cast<LinechartWidget*>(prevWidget);
        if (chart)
        {
            chart->setActive(false);
        }
    }
    QWidget* widget = plots.value(systemid, NULL);
    if (widget)
    {
        setCurrentWidget(widget);
        LinechartWidget* chart = dynamic_cast<LinechartWidget*>(widget);
        if (chart)
        {
            chart->setActive(true);
        }
    }
}

void Linecharts::addSystem(UASInterface* uas)
{
    if (!plots.contains(uas->getID()))
    {
        LinechartWidget* widget = new LinechartWidget(uas->getID(), this);
        addWidget(widget);
        plots.insert(uas->getID(), widget);
        connect(uas, SIGNAL(valueChanged(int,QString,double,quint64)), widget, SLOT(appendData(int,QString,double,quint64)));
        // Set system active if this is the only system
        if (active)
        {
            if (plots.size() == 1)
            {
                selectSystem(uas->getID());
            }
        }
    }
}
