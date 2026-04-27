package org.team100.lib.util;

import java.awt.Dimension;
import java.awt.Frame;
import java.util.List;

import javax.swing.BoxLayout;
import javax.swing.JDialog;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartColor;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.VectorRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.Vector;
import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.VectorSeriesCollection;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class ChartUtil {
    // LEAVE THIS OFF or the tests will pop up lots of windows
    public static final boolean SHOW = false;
    public static final int SIZE = 500;

    public static XYDataset xy(String name, double[] x, double[] y) {
        XYSeries series = new XYSeries(name);
        for (int i = 0; i < x.length; ++i) {
            series.add(x[i], y[i]);
        }
        return new XYSeriesCollection(series);
    }

    public static Range xRange(VectorSeriesCollection dataset) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.getItemCount(0); ++i) {
            double x = dataset.getXValue(0, i);
            Vector v = dataset.getVector(0, i);
            if (x + 0.1 > max)
                max = x + 0.1;
            if (x + v.getX() + 0.1 > max)
                max = x + v.getX() + 0.1;
            if (x - 0.1 < min)
                min = x - 0.1;
            if (x + v.getX() - 0.1 < min)
                min = x + v.getX() - 0.1;
        }
        return new Range(min, max);
    }

    public static Range yRange(VectorSeriesCollection dataset) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.getItemCount(0); ++i) {
            double y = dataset.getYValue(0, i);
            Vector v = dataset.getVector(0, i);
            if (y + 0.1 > max)
                max = y + 0.1;
            if (y + v.getY() + 0.1 > max)
                max = y + v.getY() + 0.1;
            if (y - 0.1 < min)
                min = y - 0.1;
            if (y + v.getY() - 0.1 < min)
                min = y + v.getY() - 0.1;
        }
        return new Range(min, max);
    }

    public static Range xRange(XYDataset dataset) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.getItemCount(0); ++i) {
            double x = dataset.getXValue(0, i);
            if (x + 0.1 > max)
                max = x + 0.1;
            if (x - 0.1 < min)
                min = x - 0.1;
        }
        return new Range(min, max);
    }

    public static Range yRange(XYDataset dataset) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.getItemCount(0); ++i) {
            double y = dataset.getYValue(0, i);
            if (y + 0.1 > max)
                max = y + 0.1;
            if (y - 0.1 < min)
                min = y - 0.1;
        }
        return new Range(min, max);
    }

    public static VectorSeriesCollection getDataSet(VectorSeries series) {
        VectorSeriesCollection dataSet = new VectorSeriesCollection();
        dataSet.addSeries(series);
        return dataSet;
    }

    /** plot each xy dataset as its own chart in a vertical list. */
    public static void plotStacked(XYDataset... dataSets) {
        if (!SHOW)
            return;

        // "true" means "modal" so wait for close.
        JDialog frame = new JDialog((Frame) null, "plot", true);
        frame.setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.Y_AXIS));
        for (XYDataset dataSet : dataSets) {
            XYItemRenderer renderer = new StandardXYItemRenderer(StandardXYItemRenderer.SHAPES);
            XYPlot plot = new XYPlot(
                    dataSet, new NumberAxis("X"), new NumberAxis("Y"), renderer);

            NumberAxis domain = (NumberAxis) plot.getDomainAxis();
            NumberAxis range = (NumberAxis) plot.getRangeAxis();
            domain.setRangeWithMargins(xRange(dataSet));
            range.setRangeWithMargins(yRange(dataSet));

            ChartPanel panel = new ChartPanel(new JFreeChart(plot));
            panel.setPreferredSize(new Dimension(SIZE, SIZE / dataSets.length));
            frame.add(panel);
        }

        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    }

    /** Plot multiple vector series on the same axes. */
    public static void plotOverlay(List<VectorSeries> seriesList, double scale) {
        if (!SHOW)
            return;

        // "true" means "modal" so wait for close.
        JDialog frame = new JDialog((Frame) null, "plot", true);
        frame.setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.Y_AXIS));

        XYPlot plot = new XYPlot();

        plot.setDomainAxis(new NumberAxis("X"));
        plot.setRangeAxis(new NumberAxis("Y"));

        Range xRange = null;
        Range yRange = null;
        for (int i = 0; i < seriesList.size(); ++i) {
            VectorSeries series = seriesList.get(i);
            VectorSeriesCollection dataSet = getDataSet(series);
            plot.setDataset(i, dataSet);
            XYItemRenderer renderer = new VectorRenderer();
            renderer.setSeriesPaint(0,
                    ChartColor.createDarkerColorArray(
                            ChartColor.createDefaultColorArray())[i]);
            plot.setRenderer(i, renderer);
            if (xRange == null) {
                xRange = xRange(dataSet);
                yRange = yRange(dataSet);
            } else {
                xRange = Range.combine(xRange, xRange(dataSet));
                yRange = Range.combine(yRange, yRange(dataSet));
            }
        }

        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        domain.setRangeWithMargins(xRange);
        range.setRangeWithMargins(yRange);

        JFreeChart chart = new JFreeChart(null, null, plot, false);
        ChartPanel panel = new ChartPanel(chart);
        panel.setPreferredSize(new Dimension(
                (int) (xRange.getLength() * scale),
                (int) (yRange.getLength() * scale)));
        frame.add(panel);

        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);

    }

}
