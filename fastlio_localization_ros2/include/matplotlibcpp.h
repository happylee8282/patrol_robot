#pragma once

// Python headers must be included before any system headers, since
// they define _POSIX_C_SOURCE
#include <Python.h>

#include <vector>
#include <map>
#include <array>
#include <numeric>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <cstdint> // <cstdint> requires c++11 support
#include <functional>

#ifndef WITHOUT_NUMPY
#  define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#  include <numpy/arrayobject.h>

#  ifdef WITH_OPENCV
#    include <opencv2/opencv.hpp>
#  endif // WITH_OPENCV

/*
 * A bunch of constants were removed in OpenCV 4 in favour of enum classes, so
 * define the ones we need here.
 */
#  if CV_MAJOR_VERSION > 3
#    define CV_BGR2RGB cv::COLOR_BGR2RGB
#    define CV_BGRA2RGBA cv::COLOR_BGRA2RGBA
#  endif
#endif // WITHOUT_NUMPY

#if PY_MAJOR_VERSION >= 3
#  define PyString_FromString PyUnicode_FromString
#  define PyInt_FromLong PyLong_FromLong
#  define PyString_FromString PyUnicode_FromString
#endif


namespace matplotlibcpp {
namespace detail {

static std::string s_backend;

struct _interpreter {
    PyObject* s_python_function_arrow;
    PyObject *s_python_function_show;
    PyObject *s_python_function_close;
    PyObject *s_python_function_draw;
    PyObject *s_python_function_pause;
    PyObject *s_python_function_save;
    PyObject *s_python_function_figure;
    PyObject *s_python_function_fignum_exists;
    PyObject *s_python_function_plot;
    PyObject *s_python_function_quiver;
    PyObject* s_python_function_contour;
    PyObject *s_python_function_semilogx;
    PyObject *s_python_function_semilogy;
    PyObject *s_python_function_loglog;
    PyObject *s_python_function_fill;
    PyObject *s_python_function_fill_between;
    PyObject *s_python_function_hist;
    PyObject *s_python_function_imshow;
    PyObject *s_python_function_scatter;
    PyObject *s_python_function_boxplot;
    PyObject *s_python_function_subplot;
    PyObject *s_python_function_subplot2grid;
    PyObject *s_python_function_legend;
    PyObject *s_python_function_xlim;
    PyObject *s_python_function_ion;
    PyObject *s_python_function_ginput;
    PyObject *s_python_function_ylim;
    PyObject *s_python_function_title;
    PyObject *s_python_function_axis;
    PyObject *s_python_function_axvline;
    PyObject *s_python_function_axvspan;
    PyObject *s_python_function_xlabel;
    PyObject *s_python_function_ylabel;
    PyObject *s_python_function_gca;
    PyObject *s_python_function_xticks;
    PyObject *s_python_function_yticks;
    PyObject* s_python_function_margins;
    PyObject *s_python_function_tick_params;
    PyObject *s_python_function_grid;
    PyObject* s_python_function_cla;
    PyObject *s_python_function_clf;
    PyObject *s_python_function_errorbar;
    PyObject *s_python_function_annotate;
    PyObject *s_python_function_tight_layout;
    PyObject *s_python_colormap;
    PyObject *s_python_empty_tuple;
    PyObject *s_python_function_stem;
    PyObject *s_python_function_xkcd;
    PyObject *s_python_function_text;
    PyObject *s_python_function_suptitle;
    PyObject *s_python_function_bar;
    PyObject *s_python_function_colorbar;
    PyObject *s_python_function_subplots_adjust;


    /* For now, _interpreter is implemented as a singleton since its currently not possible to have
       multiple independent embedded python interpreters without patching the python source code
       or starting a separate process for each.
        http://bytes.com/topic/python/answers/793370-multiple-independent-python-interpreters-c-c-program
       */

    static _interpreter& get() {
        static _interpreter ctx;
        return ctx;
    }

    PyObject* safe_import(PyObject* module, std::string fname) {
        PyObject* fn = PyObject_GetAttrString(module, fname.c_str());

        if (!fn)
            throw std::runtime_error(std::string("Couldn't find required function: ") + fname);

        if (!PyFunction_Check(fn))
            throw std::runtime_error(fname + std::string(" is unexpectedly not a PyFunction."));

        return fn;
    }

private:

#ifndef WITHOUT_NUMPY
#  if PY_MAJOR_VERSION >= 3

    void *import_numpy() {
        import_array(); // initialize C-API
        return NULL;
    }

#  else

    void import_numpy() {
        import_array(); // initialize C-API
    }

#  endif
#endif

    _interpreter() {

        // optional but recommended
#if PY_MAJOR_VERSION >= 3
        wchar_t name[] = L"plotting";
#else
        char name[] = "plotting";
#endif
        Py_SetProgramName(name);
        Py_Initialize();

#ifndef WITHOUT_NUMPY
        import_numpy(); // initialize numpy C-API
#endif

        PyObject* matplotlibname = PyString_FromString("matplotlib");
        PyObject* pyplotname = PyString_FromString("matplotlib.pyplot");
        PyObject* cmname  = PyString_FromString("matplotlib.cm");
        PyObject* pylabname  = PyString_FromString("pylab");
        if (!pyplotname || !pylabname || !matplotlibname || !cmname) {
            throw std::runtime_error("couldnt create string");
        }

        PyObject* matplotlib = PyImport_Import(matplotlibname);
        Py_DECREF(matplotlibname);
        if (!matplotlib) {
            PyErr_Print();
            throw std::runtime_error("Error loading module matplotlib!");
        }

        // matplotlib.use() must be called *before* pylab, matplotlib.pyplot,
        // or matplotlib.backends is imported for the first time
        if (!s_backend.empty()) {
            PyObject_CallMethod(matplotlib, const_cast<char*>("use"), const_cast<char*>("s"), s_backend.c_str());
        }

        PyObject* pymod = PyImport_Import(pyplotname);
        Py_DECREF(pyplotname);
        if (!pymod) { throw std::runtime_error("Error loading module matplotlib.pyplot!"); }

        s_python_colormap = PyImport_Import(cmname);
        Py_DECREF(cmname);
        if (!s_python_colormap) { throw std::runtime_error("Error loading module matplotlib.cm!"); }

        PyObject* pylabmod = PyImport_Import(pylabname);
        Py_DECREF(pylabname);
        if (!pylabmod) { throw std::runtime_error("Error loading module pylab!"); }

        s_python_function_arrow = safe_import(pymod, "arrow");
        s_python_function_show = safe_import(pymod, "show");
        s_python_function_close = safe_import(pymod, "close");
        s_python_function_draw = safe_import(pymod, "draw");
        s_python_function_pause = safe_import(pymod, "pause");
        s_python_function_figure = safe_import(pymod, "figure");
        s_python_function_fignum_exists = safe_import(pymod, "fignum_exists");
        s_python_function_plot = safe_import(pymod, "plot");
        s_python_function_quiver = safe_import(pymod, "quiver");
        s_python_function_contour = safe_import(pymod, "contour");
        s_python_function_semilogx = safe_import(pymod, "semilogx");
        s_python_function_semilogy = safe_import(pymod, "semilogy");
        s_python_function_loglog = safe_import(pymod, "loglog");
        s_python_function_fill = safe_import(pymod, "fill");
        s_python_function_fill_between = safe_import(pymod, "fill_between");
        s_python_function_hist = safe_import(pymod,"hist");
        s_python_function_scatter = safe_import(pymod,"scatter");
        s_python_function_boxplot = safe_import(pymod,"boxplot");
        s_python_function_subplot = safe_import(pymod, "subplot");
        s_python_function_subplot2grid = safe_import(pymod, "subplot2grid");
        s_python_function_legend = safe_import(pymod, "legend");
        s_python_function_ylim = safe_import(pymod, "ylim");
        s_python_function_title = safe_import(pymod, "title");
        s_python_function_axis = safe_import(pymod, "axis");
        s_python_function_axvline = safe_import(pymod, "axvline");
        s_python_function_axvspan = safe_import(pymod, "axvspan");
        s_python_function_xlabel = safe_import(pymod, "xlabel");
        s_python_function_ylabel = safe_import(pymod, "ylabel");
        s_python_function_gca = safe_import(pymod, "gca");
        s_python_function_xticks = safe_import(pymod, "xticks");
        s_python_function_yticks = safe_import(pymod, "yticks");
        s_python_function_margins = safe_import(pymod, "margins");
        s_python_function_tick_params = safe_import(pymod, "tick_params");
        s_python_function_grid = safe_import(pymod, "grid");
        s_python_function_xlim = safe_import(pymod, "xlim");
        s_python_function_ion = safe_import(pymod, "ion");
        s_python_function_ginput = safe_import(pymod, "ginput");
        s_python_function_save = safe_import(pylabmod, "savefig");
        s_python_function_annotate = safe_import(pymod,"annotate");
        s_python_function_cla = safe_import(pymod, "cla");
        s_python_function_clf = safe_import(pymod, "clf");
        s_python_function_errorbar = safe_import(pymod, "errorbar");
        s_python_function_tight_layout = safe_import(pymod, "tight_layout");
        s_python_function_stem = safe_import(pymod, "stem");
        s_python_function_xkcd = safe_import(pymod, "xkcd");
        s_python_function_text = safe_import(pymod, "text");
        s_python_function_suptitle = safe_import(pymod, "suptitle");
        s_python_function_bar = safe_import(pymod,"bar");
        s_python_function_colorbar = PyObject_GetAttrString(pymod, "colorbar");
        s_python_function_subplots_adjust = safe_import(pymod,"subplots_adjust");
#ifndef WITHOUT_NUMPY
        s_python_function_imshow = safe_import(pymod, "imshow");
#endif
        s_python_empty_tuple = PyTuple_New(0);
    }

    ~_interpreter() {
        Py_Finalize();
    }
};

} // end namespace detail

/// Select the backend
///
/// **NOTE:** This must be called before the first plot command to have
/// any effect.
///
/// Mainly useful to select the non-interactive 'Agg' backend when running
/// matplotlibcpp in headless mode, for example on a machine with no display.
///
/// See also: https://matplotlib.org/2.0.2/api/matplotlib_configuration_api.html#matplotlib.use
inline void backend(const std::string& name)
{
    detail::s_backend = name;
}

/**
 * @brief 在指定位置添加注释箭头和文本
 * @param annotation 注释文本
 * @param x 箭头尖端X坐标
 * @param y 箭头尖端Y坐标
 * @return 如果注释创建成功返回true
 * @details 在点(x,y)处添加文本注释，并用箭头指向该点。
 *          常用于标注图形上的特定特征点。
 *          箭头从数据点指向注释文本，默认箭头起点在数据点。
 */
inline bool annotate(std::string annotation, double x, double y)
{
    detail::_interpreter::get();

    PyObject * xy = PyTuple_New(2);
    PyObject * str = PyString_FromString(annotation.c_str());

    PyTuple_SetItem(xy,0,PyFloat_FromDouble(x));
    PyTuple_SetItem(xy,1,PyFloat_FromDouble(y));

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "xy", xy);

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, str);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_annotate, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);

    if(res) Py_DECREF(res);

    return res;
}

namespace detail {

#ifndef WITHOUT_NUMPY
// Type selector for numpy array conversion
template <typename T> struct select_npy_type { const static NPY_TYPES type = NPY_NOTYPE; }; //Default
template <> struct select_npy_type<double> { const static NPY_TYPES type = NPY_DOUBLE; };
template <> struct select_npy_type<float> { const static NPY_TYPES type = NPY_FLOAT; };
template <> struct select_npy_type<bool> { const static NPY_TYPES type = NPY_BOOL; };
template <> struct select_npy_type<int8_t> { const static NPY_TYPES type = NPY_INT8; };
template <> struct select_npy_type<int16_t> { const static NPY_TYPES type = NPY_SHORT; };
template <> struct select_npy_type<int32_t> { const static NPY_TYPES type = NPY_INT; };
template <> struct select_npy_type<int64_t> { const static NPY_TYPES type = NPY_INT64; };
template <> struct select_npy_type<uint8_t> { const static NPY_TYPES type = NPY_UINT8; };
template <> struct select_npy_type<uint16_t> { const static NPY_TYPES type = NPY_USHORT; };
template <> struct select_npy_type<uint32_t> { const static NPY_TYPES type = NPY_ULONG; };
template <> struct select_npy_type<uint64_t> { const static NPY_TYPES type = NPY_UINT64; };

// Sanity checks; comment them out or change the numpy type below if you're compiling on
// a platform where they don't apply
static_assert(sizeof(long long) == 8);
template <> struct select_npy_type<long long> { const static NPY_TYPES type = NPY_INT64; };
static_assert(sizeof(unsigned long long) == 8);
template <> struct select_npy_type<unsigned long long> { const static NPY_TYPES type = NPY_UINT64; };
// TODO: add int, long, etc.

template<typename Numeric>
PyObject* get_array(const std::vector<Numeric>& v)
{
    npy_intp vsize = v.size();
    NPY_TYPES type = select_npy_type<Numeric>::type;
    if (type == NPY_NOTYPE) {
        size_t memsize = v.size()*sizeof(double);
        double* dp = static_cast<double*>(::malloc(memsize));
        for (size_t i=0; i<v.size(); ++i)
            dp[i] = v[i];
        PyObject* varray = PyArray_SimpleNewFromData(1, &vsize, NPY_DOUBLE, dp);
        PyArray_UpdateFlags(reinterpret_cast<PyArrayObject*>(varray), NPY_ARRAY_OWNDATA);
        return varray;
    }
    
    PyObject* varray = PyArray_SimpleNewFromData(1, &vsize, type, (void*)(v.data()));
    return varray;
    }
}

/**
 * @brief 创建茎图（带关键字参数版本）
 * @tparam Numeric 数值类型
 * @param x X坐标向量
 * @param y Y值向量
 * @param keywords matplotlib关键字参数字典
 * @return 如果茎图创建成功返回true
 * @details 从(x[i],0)到(x[i],y[i])绘制垂直线，顶部有标记。
 *          使用关键字参数控制样式（如"linewidth"、"marker"、"color"）。
 */
template<typename Numeric>
bool stem(const std::vector<Numeric> &x, const std::vector<Numeric> &y, const std::map<std::string, std::string>& keywords)
{
    assert(x.size() == y.size());

    detail::_interpreter::get();

    // using numpy arrays
    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    // construct positional args
    PyObject* args = PyTuple_New(2);
    PyTuple_SetItem(args, 0, xarray);
    PyTuple_SetItem(args, 1, yarray);

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for (std::map<std::string, std::string>::const_iterator it =
            keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(),
                PyString_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(
            detail::_interpreter::get().s_python_function_stem, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);
    if (res)
        Py_DECREF(res);

    return res;
}

template< typename Numeric >
bool fill(const std::vector<Numeric>& x, const std::vector<Numeric>& y, const std::map<std::string, std::string>& keywords)
{
    assert(x.size() == y.size());

    detail::_interpreter::get();

    // using numpy arrays
    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    // construct positional args
    PyObject* args = PyTuple_New(2);
    PyTuple_SetItem(args, 0, xarray);
    PyTuple_SetItem(args, 1, yarray);

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_fill, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);

    if (res) Py_DECREF(res);

    return res;
}

template< typename Numeric >
bool fill_between(const std::vector<Numeric>& x, const std::vector<Numeric>& y1, const std::vector<Numeric>& y2, const std::map<std::string, std::string>& keywords)
{
    assert(x.size() == y1.size());
    assert(x.size() == y2.size());

    detail::_interpreter::get();

    // using numpy arrays
    PyObject* xarray = detail::get_array(x);
    PyObject* y1array = detail::get_array(y1);
    PyObject* y2array = detail::get_array(y2);

    // construct positional args
    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, xarray);
    PyTuple_SetItem(args, 1, y1array);
    PyTuple_SetItem(args, 2, y2array);

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_fill_between, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);
    if(res) Py_DECREF(res);

    return res;
}

template <typename Numeric>
bool arrow(Numeric x, Numeric y, Numeric end_x, Numeric end_y, const std::string& fc = "r",
           const std::string ec = "k", Numeric head_length = 0.25, Numeric head_width = 0.1625) {
    PyObject* obj_x = PyFloat_FromDouble(x);
    PyObject* obj_y = PyFloat_FromDouble(y);
    PyObject* obj_end_x = PyFloat_FromDouble(end_x);
    PyObject* obj_end_y = PyFloat_FromDouble(end_y);

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "fc", PyString_FromString(fc.c_str()));
    PyDict_SetItemString(kwargs, "ec", PyString_FromString(ec.c_str()));
    PyDict_SetItemString(kwargs, "head_width", PyFloat_FromDouble(head_width));
    PyDict_SetItemString(kwargs, "head_length", PyFloat_FromDouble(head_length));

    PyObject* plot_args = PyTuple_New(4);
    PyTuple_SetItem(plot_args, 0, obj_x);
    PyTuple_SetItem(plot_args, 1, obj_y);
    PyTuple_SetItem(plot_args, 2, obj_end_x);
    PyTuple_SetItem(plot_args, 3, obj_end_y);

    PyObject* res =
            PyObject_Call(detail::_interpreter::get().s_python_function_arrow, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if (res)
        Py_DECREF(res);

    return res;
}

template< typename Numeric>
bool hist(const std::vector<Numeric>& y, long bins=10,std::string color="b",
          double alpha=1.0, bool cumulative=false)
{
    detail::_interpreter::get();

    PyObject* yarray = detail::get_array(y);

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "bins", PyLong_FromLong(bins));
    PyDict_SetItemString(kwargs, "color", PyString_FromString(color.c_str()));
    PyDict_SetItemString(kwargs, "alpha", PyFloat_FromDouble(alpha));
    PyDict_SetItemString(kwargs, "cumulative", cumulative ? Py_True : Py_False);

    PyObject* plot_args = PyTuple_New(1);

    PyTuple_SetItem(plot_args, 0, yarray);


    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_hist, plot_args, kwargs);


    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if(res) Py_DECREF(res);

    return res;
}

#ifndef WITHOUT_NUMPY
namespace detail {

inline void imshow(void *ptr, const NPY_TYPES type, const int rows, const int columns, const int colors, const std::map<std::string, std::string> &keywords, PyObject** out)
{
    assert(type == NPY_UINT8 || type == NPY_FLOAT);
    assert(colors == 1 || colors == 3 || colors == 4);

    detail::_interpreter::get();

    // construct args
    npy_intp dims[3] = { rows, columns, colors };
    PyObject *args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, PyArray_SimpleNewFromData(colors == 1 ? 2 : 3, dims, type, ptr));

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject *res = PyObject_Call(detail::_interpreter::get().s_python_function_imshow, args, kwargs);
    Py_DECREF(args);
    Py_DECREF(kwargs);
    if (!res)
        throw std::runtime_error("Call to imshow() failed");
    if (out)
        *out = res;
    else
        Py_DECREF(res);
}

} // namespace detail

inline void imshow(const unsigned char *ptr, const int rows, const int columns, const int colors, const std::map<std::string, std::string> &keywords = {}, PyObject** out = nullptr)
{
    detail::imshow((void *) ptr, NPY_UINT8, rows, columns, colors, keywords, out);
}

inline void imshow(const float *ptr, const int rows, const int columns, const int colors, const std::map<std::string, std::string> &keywords = {}, PyObject** out = nullptr)
{
    detail::imshow((void *) ptr, NPY_FLOAT, rows, columns, colors, keywords, out);
}

#ifdef WITH_OPENCV
void imshow(const cv::Mat &image, const std::map<std::string, std::string> &keywords = {})
{
    // Convert underlying type of matrix, if needed
    cv::Mat image2;
    NPY_TYPES npy_type = NPY_UINT8;
    switch (image.type() & CV_MAT_DEPTH_MASK) {
    case CV_8U:
        image2 = image;
        break;
    case CV_32F:
        image2 = image;
        npy_type = NPY_FLOAT;
        break;
    default:
        image.convertTo(image2, CV_MAKETYPE(CV_8U, image.channels()));
    }

    // If color image, convert from BGR to RGB
    switch (image2.channels()) {
    case 3:
        cv::cvtColor(image2, image2, CV_BGR2RGB);
        break;
    case 4:
        cv::cvtColor(image2, image2, CV_BGRA2RGBA);
    }

    detail::imshow(image2.data, npy_type, image2.rows, image2.cols, image2.channels(), keywords);
}
#endif // WITH_OPENCV
#endif // WITHOUT_NUMPY

template<typename NumericX, typename NumericY>
bool scatter(const std::vector<NumericX>& x,
             const std::vector<NumericY>& y,
             const double s=1.0, // The marker size in points**2
             const std::map<std::string, std::string> & keywords = {})
{
    detail::_interpreter::get();

    assert(x.size() == y.size());

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "s", PyLong_FromLong(s));
    for (const auto& it : keywords)
    {
        PyDict_SetItemString(kwargs, it.first.c_str(), PyString_FromString(it.second.c_str()));
    }

    PyObject* plot_args = PyTuple_New(2);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_scatter, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if(res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool boxplot(const std::vector<std::vector<Numeric>>& data,
             const std::vector<std::string>& labels = {},
             const std::map<std::string, std::string> & keywords = {})
{
    detail::_interpreter::get();

    PyObject* listlist = detail::get_listlist(data);
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, listlist);

    PyObject* kwargs = PyDict_New();

    // kwargs needs the labels, if there are (the correct number of) labels
    if (!labels.empty() && labels.size() == data.size()) {
        PyDict_SetItemString(kwargs, "labels", detail::get_array(labels));
    }

    // take care of the remaining keywords
    for (const auto& it : keywords)
    {
        PyDict_SetItemString(kwargs, it.first.c_str(), PyString_FromString(it.second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_boxplot, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);

    if(res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool boxplot(const std::vector<Numeric>& data,
             const std::map<std::string, std::string> & keywords = {})
{
    detail::_interpreter::get();

    PyObject* vector = detail::get_array(data);
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, vector);

    PyObject* kwargs = PyDict_New();
    for (const auto& it : keywords)
    {
        PyDict_SetItemString(kwargs, it.first.c_str(), PyString_FromString(it.second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_boxplot, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);

    if(res) Py_DECREF(res);

    return res;
}

template <typename Numeric>
bool bar(const std::vector<Numeric> &               x,
         const std::vector<Numeric> &               y,
         std::string                                ec       = "black",
         std::string                                ls       = "-",
         double                                     lw       = 1.0,
         const std::map<std::string, std::string> & keywords = {})
{
  detail::_interpreter::get();

  PyObject * xarray = detail::get_array(x);
  PyObject * yarray = detail::get_array(y);

  PyObject * kwargs = PyDict_New();

  PyDict_SetItemString(kwargs, "ec", PyString_FromString(ec.c_str()));
  PyDict_SetItemString(kwargs, "ls", PyString_FromString(ls.c_str()));
  PyDict_SetItemString(kwargs, "lw", PyFloat_FromDouble(lw));

  for (std::map<std::string, std::string>::const_iterator it =
         keywords.begin();
       it != keywords.end();
       ++it) {
    PyDict_SetItemString(
      kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
  }

  PyObject * plot_args = PyTuple_New(2);
  PyTuple_SetItem(plot_args, 0, xarray);
  PyTuple_SetItem(plot_args, 1, yarray);

  PyObject * res = PyObject_Call(
    detail::_interpreter::get().s_python_function_bar, plot_args, kwargs);

  Py_DECREF(plot_args);
  Py_DECREF(kwargs);
  if (res) Py_DECREF(res);

  return res;
}

template <typename Numeric>
bool bar(const std::vector<Numeric> &               y,
         std::string                                ec       = "black",
         std::string                                ls       = "-",
         double                                     lw       = 1.0,
         const std::map<std::string, std::string> & keywords = {})
{
  using T = typename std::remove_reference<decltype(y)>::type::value_type;

  detail::_interpreter::get();

  std::vector<T> x;
  for (std::size_t i = 0; i < y.size(); i++) { x.push_back(i); }

  return bar(x, y, ec, ls, lw, keywords);
}

inline bool subplots_adjust(const std::map<std::string, double>& keywords = {})
{
    detail::_interpreter::get();

    PyObject* kwargs = PyDict_New();
    for (std::map<std::string, double>::const_iterator it =
            keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(),
                             PyFloat_FromDouble(it->second));
    }


    PyObject* plot_args = PyTuple_New(0);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_subplots_adjust, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if(res) Py_DECREF(res);

    return res;
}

template< typename Numeric>
bool named_hist(std::string label,const std::vector<Numeric>& y, long bins=10, std::string color="b", double alpha=1.0)
{
    detail::_interpreter::get();

    PyObject* yarray = detail::get_array(y);

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "label", PyString_FromString(label.c_str()));
    PyDict_SetItemString(kwargs, "bins", PyLong_FromLong(bins));
    PyDict_SetItemString(kwargs, "color", PyString_FromString(color.c_str()));
    PyDict_SetItemString(kwargs, "alpha", PyFloat_FromDouble(alpha));


    PyObject* plot_args = PyTuple_New(1);
    PyTuple_SetItem(plot_args, 0, yarray);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_hist, plot_args, kwargs);

    Py_DECREF(plot_args);
    Py_DECREF(kwargs);
    if(res) Py_DECREF(res);

    return res;
}

template<typename NumericX, typename NumericY>
bool plot(const std::vector<NumericX>& x, const std::vector<NumericY>& y, const std::string& s = "")
{
    assert(x.size() == y.size());

    detail::_interpreter::get();

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(s.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_plot, plot_args);

    Py_DECREF(plot_args);
    if(res) Py_DECREF(res);

    return res;
}

template <typename NumericX, typename NumericY, typename NumericZ>
bool contour(const std::vector<NumericX>& x, const std::vector<NumericY>& y,
             const std::vector<NumericZ>& z,
             const std::map<std::string, std::string>& keywords = {}) {
    assert(x.size() == y.size() && x.size() == z.size());

    PyObject* xarray = get_array(x);
    PyObject* yarray = get_array(y);
    PyObject* zarray = get_array(z);

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, zarray);

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for (std::map<std::string, std::string>::const_iterator it = keywords.begin();
         it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res =
            PyObject_Call(detail::_interpreter::get().s_python_function_contour, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);
    if (res)
        Py_DECREF(res);

    return res;
}

template<typename NumericX, typename NumericY, typename NumericU, typename NumericW>
bool quiver(const std::vector<NumericX>& x, const std::vector<NumericY>& y, const std::vector<NumericU>& u, const std::vector<NumericW>& w, const std::map<std::string, std::string>& keywords = {})
{
    assert(x.size() == y.size() && x.size() == u.size() && u.size() == w.size());

    detail::_interpreter::get();

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);
    PyObject* uarray = detail::get_array(u);
    PyObject* warray = detail::get_array(w);

    PyObject* plot_args = PyTuple_New(4);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, uarray);
    PyTuple_SetItem(plot_args, 3, warray);

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(
            detail::_interpreter::get().s_python_function_quiver, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);
    if (res)
        Py_DECREF(res);

    return res;
}

template<typename NumericX, typename NumericY>
bool stem(const std::vector<NumericX>& x, const std::vector<NumericY>& y, const std::string& s = "")
{
    assert(x.size() == y.size());

    detail::_interpreter::get();

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(s.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_CallObject(
            detail::_interpreter::get().s_python_function_stem, plot_args);

    Py_DECREF(plot_args);
    if (res)
        Py_DECREF(res);

    return res;
}

template<typename NumericX, typename NumericY>
bool semilogx(const std::vector<NumericX>& x, const std::vector<NumericY>& y, const std::string& s = "")
{
    assert(x.size() == y.size());

    detail::_interpreter::get();

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(s.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_semilogx, plot_args);

    Py_DECREF(plot_args);
    if(res) Py_DECREF(res);

    return res;
}

template<typename NumericX, typename NumericY>
bool semilogy(const std::vector<NumericX>& x, const std::vector<NumericY>& y, const std::string& s = "")
{
    assert(x.size() == y.size());

    detail::_interpreter::get();

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(s.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_semilogy, plot_args);

    Py_DECREF(plot_args);
    if(res) Py_DECREF(res);

    return res;
}

template<typename NumericX, typename NumericY>
bool loglog(const std::vector<NumericX>& x, const std::vector<NumericY>& y, const std::string& s = "")
{
    assert(x.size() == y.size());

    detail::_interpreter::get();

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(s.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_loglog, plot_args);

    Py_DECREF(plot_args);
    if(res) Py_DECREF(res);

    return res;
}

template<typename NumericX, typename NumericY>
bool errorbar(const std::vector<NumericX> &x, const std::vector<NumericY> &y, const std::vector<NumericX> &yerr, const std::map<std::string, std::string> &keywords = {})
{
    assert(x.size() == y.size());

    detail::_interpreter::get();

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);
    PyObject* yerrarray = detail::get_array(yerr);

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyDict_SetItemString(kwargs, "yerr", yerrarray);

    PyObject *plot_args = PyTuple_New(2);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);

    PyObject *res = PyObject_Call(detail::_interpreter::get().s_python_function_errorbar, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);

    if (res)
        Py_DECREF(res);
    else
        throw std::runtime_error("Call to errorbar() failed.");

    return res;
}

template<typename Numeric>
bool named_plot(const std::string& name, const std::vector<Numeric>& y, const std::string& format = "")
{
    detail::_interpreter::get();

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(format.c_str());

    PyObject* plot_args = PyTuple_New(2);

    PyTuple_SetItem(plot_args, 0, yarray);
    PyTuple_SetItem(plot_args, 1, pystring);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_plot, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);
    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool named_plot(const std::string& name, const std::vector<Numeric>& x, const std::vector<Numeric>& y, const std::string& format = "")
{
    detail::_interpreter::get();

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(format.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_plot, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);
    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool named_semilogx(const std::string& name, const std::vector<Numeric>& x, const std::vector<Numeric>& y, const std::string& format = "")
{
    detail::_interpreter::get();

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(format.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_semilogx, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);
    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool named_semilogy(const std::string& name, const std::vector<Numeric>& x, const std::vector<Numeric>& y, const std::string& format = "")
{
    detail::_interpreter::get();

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(format.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_semilogy, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);
    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool named_loglog(const std::string& name, const std::vector<Numeric>& x, const std::vector<Numeric>& y, const std::string& format = "")
{
    detail::_interpreter::get();

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

    PyObject* xarray = detail::get_array(x);
    PyObject* yarray = detail::get_array(y);

    PyObject* pystring = PyString_FromString(format.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);
    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_loglog, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);
    if (res) Py_DECREF(res);

    return res;
}

template<typename Numeric>
bool plot(const std::vector<Numeric>& y, const std::string& format = "")
{
    std::vector<Numeric> x(y.size());
    for(size_t i=0; i<x.size(); ++i) x.at(i) = i;
    return plot(x,y,format);
}

template<typename Numeric>
bool plot(const std::vector<Numeric>& y, const std::map<std::string, std::string>& keywords)
{
    std::vector<Numeric> x(y.size());
    for(size_t i=0; i<x.size(); ++i) x.at(i) = i;
    return plot(x,y,keywords);
}

template<typename Numeric>
bool stem(const std::vector<Numeric>& y, const std::string& format = "")
{
    std::vector<Numeric> x(y.size());
    for (size_t i = 0; i < x.size(); ++i) x.at(i) = i;
    return stem(x, y, format);
}

template<typename Numeric>
void text(Numeric x, Numeric y, const std::string& s = "")
{
    detail::_interpreter::get();

    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(x));
    PyTuple_SetItem(args, 1, PyFloat_FromDouble(y));
    PyTuple_SetItem(args, 2, PyString_FromString(s.c_str()));

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_text, args);
    if(!res) throw std::runtime_error("Call to text() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}

inline void colorbar(PyObject* mappable = NULL, const std::map<std::string, float>& keywords = {})
{
    if (mappable == NULL)
        throw std::runtime_error("Must call colorbar with PyObject* returned from an image, contour, surface, etc.");

    detail::_interpreter::get();

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, mappable);

    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, float>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyFloat_FromDouble(it->second));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_colorbar, args, kwargs);
    if(!res) throw std::runtime_error("Call to colorbar() failed.");

    Py_DECREF(args);
    Py_DECREF(kwargs);
    Py_DECREF(res);
}


inline long figure(long number = -1)
{
    detail::_interpreter::get();

    PyObject *res;
    if (number == -1)
        res = PyObject_CallObject(detail::_interpreter::get().s_python_function_figure, detail::_interpreter::get().s_python_empty_tuple);
    else {
        assert(number > 0);

        // Make sure interpreter is initialised
        detail::_interpreter::get();

        PyObject *args = PyTuple_New(1);
        PyTuple_SetItem(args, 0, PyLong_FromLong(number));
        res = PyObject_CallObject(detail::_interpreter::get().s_python_function_figure, args);
        Py_DECREF(args);
    }

    if(!res) throw std::runtime_error("Call to figure() failed.");

    PyObject* num = PyObject_GetAttrString(res, "number");
    if (!num) throw std::runtime_error("Could not get number attribute of figure object");
    const long figureNumber = PyLong_AsLong(num);

    Py_DECREF(num);
    Py_DECREF(res);

    return figureNumber;
}

inline bool fignum_exists(long number)
{
    detail::_interpreter::get();

    PyObject *args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, PyLong_FromLong(number));
    PyObject *res = PyObject_CallObject(detail::_interpreter::get().s_python_function_fignum_exists, args);
    if(!res) throw std::runtime_error("Call to fignum_exists() failed.");

    bool ret = PyObject_IsTrue(res);
    Py_DECREF(res);
    Py_DECREF(args);

    return ret;
}

inline void figure_size(size_t w, size_t h)
{
    detail::_interpreter::get();

    const size_t dpi = 100;
    PyObject* size = PyTuple_New(2);
    PyTuple_SetItem(size, 0, PyFloat_FromDouble((double)w / dpi));
    PyTuple_SetItem(size, 1, PyFloat_FromDouble((double)h / dpi));

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "figsize", size);
    PyDict_SetItemString(kwargs, "dpi", PyLong_FromSize_t(dpi));

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_figure,
            detail::_interpreter::get().s_python_empty_tuple, kwargs);

    Py_DECREF(kwargs);

    if(!res) throw std::runtime_error("Call to figure_size() failed.");
    Py_DECREF(res);
}

inline void legend()
{
    detail::_interpreter::get();

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_legend, detail::_interpreter::get().s_python_empty_tuple);
    if(!res) throw std::runtime_error("Call to legend() failed.");

    Py_DECREF(res);
}

inline void legend(const std::map<std::string, std::string>& keywords)
{
  detail::_interpreter::get();

  // construct keyword args
  PyObject* kwargs = PyDict_New();
  for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
  {
    PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
  }

  PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_legend, detail::_interpreter::get().s_python_empty_tuple, kwargs);
  if(!res) throw std::runtime_error("Call to legend() failed.");

  Py_DECREF(kwargs);
  Py_DECREF(res);  
}

template<typename Numeric>
void ylim(Numeric left, Numeric right)
{
    detail::_interpreter::get();

    PyObject* list = PyList_New(2);
    PyList_SetItem(list, 0, PyFloat_FromDouble(left));
    PyList_SetItem(list, 1, PyFloat_FromDouble(right));

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, list);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_ylim, args);
    if(!res) throw std::runtime_error("Call to ylim() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}

template<typename Numeric>
void xlim(Numeric left, Numeric right)
{
    detail::_interpreter::get();

    PyObject* list = PyList_New(2);
    PyList_SetItem(list, 0, PyFloat_FromDouble(left));
    PyList_SetItem(list, 1, PyFloat_FromDouble(right));

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, list);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_xlim, args);
    if(!res) throw std::runtime_error("Call to xlim() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}


inline double* xlim()
{
    detail::_interpreter::get();

    PyObject* args = PyTuple_New(0);
    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_xlim, args);
    PyObject* left = PyTuple_GetItem(res,0);
    PyObject* right = PyTuple_GetItem(res,1);

    double* arr = new double[2];
    arr[0] = PyFloat_AsDouble(left);
    arr[1] = PyFloat_AsDouble(right);

    if(!res) throw std::runtime_error("Call to xlim() failed.");

    Py_DECREF(res);
    return arr;
}


inline double* ylim()
{
    detail::_interpreter::get();

    PyObject* args = PyTuple_New(0);
    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_ylim, args);
    PyObject* left = PyTuple_GetItem(res,0);
    PyObject* right = PyTuple_GetItem(res,1);

    double* arr = new double[2];
    arr[0] = PyFloat_AsDouble(left);
    arr[1] = PyFloat_AsDouble(right);

    if(!res) throw std::runtime_error("Call to ylim() failed.");

    Py_DECREF(res);
    return arr;
}

template<typename Numeric>
inline void xticks(const std::vector<Numeric> &ticks, const std::vector<std::string> &labels = {}, const std::map<std::string, std::string>& keywords = {})
{
    assert(labels.size() == 0 || ticks.size() == labels.size());

    detail::_interpreter::get();

    // using numpy array
    PyObject* ticksarray = detail::get_array(ticks);

    PyObject* args;
    if(labels.size() == 0) {
        // construct positional args
        args = PyTuple_New(1);
        PyTuple_SetItem(args, 0, ticksarray);
    } else {
        // make tuple of tick labels
        PyObject* labelstuple = PyTuple_New(labels.size());
        for (size_t i = 0; i < labels.size(); i++)
            PyTuple_SetItem(labelstuple, i, PyUnicode_FromString(labels[i].c_str()));

        // construct positional args
        args = PyTuple_New(2);
        PyTuple_SetItem(args, 0, ticksarray);
        PyTuple_SetItem(args, 1, labelstuple);
    }

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_xticks, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);
    if(!res) throw std::runtime_error("Call to xticks() failed");

    Py_DECREF(res);
}

template<typename Numeric>
inline void xticks(const std::vector<Numeric> &ticks, const std::map<std::string, std::string>& keywords)
{
    xticks(ticks, {}, keywords);
}

template<typename Numeric>
inline void yticks(const std::vector<Numeric> &ticks, const std::vector<std::string> &labels = {}, const std::map<std::string, std::string>& keywords = {})
{
    assert(labels.size() == 0 || ticks.size() == labels.size());

    detail::_interpreter::get();

    // using numpy array
    PyObject* ticksarray = detail::get_array(ticks);

    PyObject* args;
    if(labels.size() == 0) {
        // construct positional args
        args = PyTuple_New(1);
        PyTuple_SetItem(args, 0, ticksarray);
    } else {
        // make tuple of tick labels
        PyObject* labelstuple = PyTuple_New(labels.size());
        for (size_t i = 0; i < labels.size(); i++)
            PyTuple_SetItem(labelstuple, i, PyUnicode_FromString(labels[i].c_str()));

        // construct positional args
        args = PyTuple_New(2);
        PyTuple_SetItem(args, 0, ticksarray);
        PyTuple_SetItem(args, 1, labelstuple);
    }

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_yticks, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);
    if(!res) throw std::runtime_error("Call to yticks() failed");

    Py_DECREF(res);
}

template<typename Numeric>
inline void yticks(const std::vector<Numeric> &ticks, const std::map<std::string, std::string>& keywords)
{
    yticks(ticks, {}, keywords);
}

template <typename Numeric> inline void margins(Numeric margin)
{
    // construct positional args
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(margin));

    PyObject* res =
            PyObject_CallObject(detail::_interpreter::get().s_python_function_margins, args);
    if (!res)
        throw std::runtime_error("Call to margins() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}

template <typename Numeric> inline void margins(Numeric margin_x, Numeric margin_y)
{
    // construct positional args
    PyObject* args = PyTuple_New(2);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(margin_x));
    PyTuple_SetItem(args, 1, PyFloat_FromDouble(margin_y));

    PyObject* res =
            PyObject_CallObject(detail::_interpreter::get().s_python_function_margins, args);
    if (!res)
        throw std::runtime_error("Call to margins() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}


inline void tick_params(const std::map<std::string, std::string>& keywords, const std::string axis = "both")
{
  detail::_interpreter::get();

  // construct positional args
  PyObject* args;
  args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, PyString_FromString(axis.c_str()));

  // construct keyword args
  PyObject* kwargs = PyDict_New();
  for (std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
  {
    PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
  }


  PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_tick_params, args, kwargs);

  Py_DECREF(args);
  Py_DECREF(kwargs);
  if (!res) throw std::runtime_error("Call to tick_params() failed");

  Py_DECREF(res);
}

inline void subplot(long nrows, long ncols, long plot_number)
{
    detail::_interpreter::get();
    
    // construct positional args
    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(nrows));
    PyTuple_SetItem(args, 1, PyFloat_FromDouble(ncols));
    PyTuple_SetItem(args, 2, PyFloat_FromDouble(plot_number));

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_subplot, args);
    if(!res) throw std::runtime_error("Call to subplot() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}

inline void subplot2grid(long nrows, long ncols, long rowid=0, long colid=0, long rowspan=1, long colspan=1)
{
    detail::_interpreter::get();

    PyObject* shape = PyTuple_New(2);
    PyTuple_SetItem(shape, 0, PyLong_FromLong(nrows));
    PyTuple_SetItem(shape, 1, PyLong_FromLong(ncols));

    PyObject* loc = PyTuple_New(2);
    PyTuple_SetItem(loc, 0, PyLong_FromLong(rowid));
    PyTuple_SetItem(loc, 1, PyLong_FromLong(colid));

    PyObject* args = PyTuple_New(4);
    PyTuple_SetItem(args, 0, shape);
    PyTuple_SetItem(args, 1, loc);
    PyTuple_SetItem(args, 2, PyLong_FromLong(rowspan));
    PyTuple_SetItem(args, 3, PyLong_FromLong(colspan));

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_subplot2grid, args);
    if(!res) throw std::runtime_error("Call to subplot2grid() failed.");

    Py_DECREF(shape);
    Py_DECREF(loc);
    Py_DECREF(args);
    Py_DECREF(res);
}

inline void title(const std::string &titlestr, const std::map<std::string, std::string> &keywords = {})
{
    detail::_interpreter::get();

    PyObject* pytitlestr = PyString_FromString(titlestr.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pytitlestr);

    PyObject* kwargs = PyDict_New();
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_title, args, kwargs);
    if(!res) throw std::runtime_error("Call to title() failed.");

    Py_DECREF(args);
    Py_DECREF(kwargs);
    Py_DECREF(res);
}

inline void suptitle(const std::string &suptitlestr, const std::map<std::string, std::string> &keywords = {})
{
    detail::_interpreter::get();
    
    PyObject* pysuptitlestr = PyString_FromString(suptitlestr.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pysuptitlestr);

    PyObject* kwargs = PyDict_New();
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_suptitle, args, kwargs);
    if(!res) throw std::runtime_error("Call to suptitle() failed.");

    Py_DECREF(args);
    Py_DECREF(kwargs);
    Py_DECREF(res);
}

inline void axis(const std::string &axisstr)
{
    detail::_interpreter::get();

    PyObject* str = PyString_FromString(axisstr.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, str);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_axis, args);
    if(!res) throw std::runtime_error("Call to title() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}

inline void axvline(double x, double ymin = 0., double ymax = 1., const std::map<std::string, std::string>& keywords = std::map<std::string, std::string>())
{
    detail::_interpreter::get();

    // construct positional args
    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(x));
    PyTuple_SetItem(args, 1, PyFloat_FromDouble(ymin));
    PyTuple_SetItem(args, 2, PyFloat_FromDouble(ymax));

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_axvline, args, kwargs);

    Py_DECREF(args);
    Py_DECREF(kwargs);

    if(res) Py_DECREF(res);
}

inline void axvspan(double xmin, double xmax, double ymin = 0., double ymax = 1., const std::map<std::string, std::string>& keywords = std::map<std::string, std::string>())
{
    // construct positional args
    PyObject* args = PyTuple_New(4);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(xmin));
    PyTuple_SetItem(args, 1, PyFloat_FromDouble(xmax));
    PyTuple_SetItem(args, 2, PyFloat_FromDouble(ymin));
    PyTuple_SetItem(args, 3, PyFloat_FromDouble(ymax));

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
	if (it->first == "linewidth" || it->first == "alpha")
    	    PyDict_SetItemString(kwargs, it->first.c_str(), PyFloat_FromDouble(std::stod(it->second)));
  	else
    	    PyDict_SetItemString(kwargs, it->first.c_str(), PyString_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_axvspan, args, kwargs);
    Py_DECREF(args);
    Py_DECREF(kwargs);

    if(res) Py_DECREF(res);
}

inline void xlabel(const std::string &str, const std::map<std::string, std::string> &keywords = {})
{
    detail::_interpreter::get();

    PyObject* pystr = PyString_FromString(str.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pystr);

    PyObject* kwargs = PyDict_New();
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_xlabel, args, kwargs);
    if(!res) throw std::runtime_error("Call to xlabel() failed.");

    Py_DECREF(args);
    Py_DECREF(kwargs);
    Py_DECREF(res);
}

inline void ylabel(const std::string &str, const std::map<std::string, std::string>& keywords = {})
{
    detail::_interpreter::get();

    PyObject* pystr = PyString_FromString(str.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pystr);

    PyObject* kwargs = PyDict_New();
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_ylabel, args, kwargs);
    if(!res) throw std::runtime_error("Call to ylabel() failed.");

    Py_DECREF(args);
    Py_DECREF(kwargs);
    Py_DECREF(res);
}

inline void set_zlabel(const std::string &str, const std::map<std::string, std::string>& keywords = {})
{
    detail::_interpreter::get();

    // Same as with plot_surface: We lazily load the modules here the first time 
    // this function is called because I'm not sure that we can assume "matplotlib 
    // installed" implies "mpl_toolkits installed" on all platforms, and we don't 
    // want to require it for people who don't need 3d plots.
    static PyObject *mpl_toolkitsmod = nullptr, *axis3dmod = nullptr;
    if (!mpl_toolkitsmod) {
        PyObject* mpl_toolkits = PyString_FromString("mpl_toolkits");
        PyObject* axis3d = PyString_FromString("mpl_toolkits.mplot3d");
        if (!mpl_toolkits || !axis3d) { throw std::runtime_error("couldnt create string"); }

        mpl_toolkitsmod = PyImport_Import(mpl_toolkits);
        Py_DECREF(mpl_toolkits);
        if (!mpl_toolkitsmod) { throw std::runtime_error("Error loading module mpl_toolkits!"); }

        axis3dmod = PyImport_Import(axis3d);
        Py_DECREF(axis3d);
        if (!axis3dmod) { throw std::runtime_error("Error loading module mpl_toolkits.mplot3d!"); }
    }

    PyObject* pystr = PyString_FromString(str.c_str());
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pystr);

    PyObject* kwargs = PyDict_New();
    for (auto it = keywords.begin(); it != keywords.end(); ++it) {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject *ax =
    PyObject_CallObject(detail::_interpreter::get().s_python_function_gca,
      detail::_interpreter::get().s_python_empty_tuple);
    if (!ax) throw std::runtime_error("Call to gca() failed.");
    Py_INCREF(ax);

    PyObject *zlabel = PyObject_GetAttrString(ax, "set_zlabel");
    if (!zlabel) throw std::runtime_error("Attribute set_zlabel not found.");
    Py_INCREF(zlabel);

    PyObject *res = PyObject_Call(zlabel, args, kwargs);
    if (!res) throw std::runtime_error("Call to set_zlabel() failed.");
    Py_DECREF(zlabel);

    Py_DECREF(ax);
    Py_DECREF(args);
    Py_DECREF(kwargs);
    if (res) Py_DECREF(res);
}

/**
 * @brief 启用或禁用网格线显示
 *
 * 该函数控制当前坐标轴是否显示网格线。网格线有助于视觉上对齐数据点，
 * 并估计数据值。网格线默认是禁用的，需要显式调用此函数启用。
 *
 * @param flag 是否显示网格线的标志
 *             - true: 启用网格线显示
 *             - false: 禁用网格线显示
 *
 * @note 网格线样式（颜色、线型、线宽）可以通过调用 grid(True) 后的
 *       rcParams 设置进行自定义，或使用 grid(True, which='both') 等
 *       更细粒度的控制（当前实现只支持基本开关）。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.grid() - Python官方文档
 */
inline void grid(bool flag)
{
    detail::_interpreter::get();

    PyObject* pyflag = flag ? Py_True : Py_False;
    Py_INCREF(pyflag);

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pyflag);

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_grid, args);
    if(!res) throw std::runtime_error("Call to grid() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}

/**
 * @brief 显示所有图形
 *
 * 该函数运行GUI事件循环，阻塞或非阻塞地显示所有已创建的图形窗口。
 * 在交互模式下，此函数通常在脚本末尾调用以保持图形窗口打开。
 * 在非交互模式下，这是显示图形的唯一方式。
 *
 * @param block 是否阻塞程序执行，默认为 true
 *              - true: 阻塞模式，函数会阻塞直到所有图形窗口被关闭
 *              - false: 非阻塞模式，函数立即返回，图形窗口在后台运行
 *
 * @note 1. 在交互模式下（ion() 已调用），使用 block=false 可以实现
 *        动态更新图形而不阻塞程序执行。
 * @note 2. 每个图形窗口需要至少一次 show() 调用才能显示。
 * @note 3. 在脚本中，通常在所有绘图命令之后调用 show()。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.show() - Python官方文档
 * @see ion() - 交互模式开关
 */
inline void show(const bool block = true)
{
    detail::_interpreter::get();

    PyObject* res;
    if(block)
    {
        res = PyObject_CallObject(
                detail::_interpreter::get().s_python_function_show,
                detail::_interpreter::get().s_python_empty_tuple);
    }
    else
    {
        PyObject *kwargs = PyDict_New();
        PyDict_SetItemString(kwargs, "block", Py_False);
        res = PyObject_Call( detail::_interpreter::get().s_python_function_show, detail::_interpreter::get().s_python_empty_tuple, kwargs);
       Py_DECREF(kwargs);
    }


    if (!res) throw std::runtime_error("Call to show() failed.");

    Py_DECREF(res);
}

/**
 * @brief 关闭当前图形窗口
 *
 * 该函数关闭当前活动的图形窗口（由 gcf() 返回）。如果存在多个图形窗口，
 * 只关闭当前活动窗口，其他窗口保持打开状态。要关闭所有图形窗口，需要
 * 循环调用此函数或使用 plt.close('all')（当前C++封装未直接支持）。
 *
 * @note 1. 关闭图形窗口后，该图形对象及其所有坐标轴、线条、文本等
 *        资源都会被释放。
 * @note 2. 关闭当前活动窗口后，Matplotlib会自动激活下一个可用窗口
 *         作为当前活动窗口。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.close() - Python官方文档
 * @see close(const char*) - Python支持通过名称或编号关闭特定窗口
 */
inline void close()
{
    detail::_interpreter::get();

    PyObject* res = PyObject_CallObject(
            detail::_interpreter::get().s_python_function_close,
            detail::_interpreter::get().s_python_empty_tuple);

    if (!res) throw std::runtime_error("Call to close() failed.");

    Py_DECREF(res);
}

/**
 * @brief 启用XKCD风格的手绘风格绘图
 *
 * 该函数启用XKCD漫画风格的绘图样式，使所有后续图形具有手绘、草图般
 * 的外观效果。这种风格模仿了Randall Munroe的XKCD漫画中的插图风格，
 * 包括不规则的线条、手写字体和简化的图形元素。
 *
 * @note 1. 此效果只影响调用此函数之后创建的图形。
 * @note 2. XKCD风格通过修改Matplotlib的rcParams全局设置实现，会影响
 *         所有后续绘图，直到调用 style.use() 或其他样式设置覆盖。
 * @note 3. 可使用 with plt.xkcd(): 上下文管理器（当前C++封装未直接支持）
 *         临时启用此效果。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.xkcd() - Python官方文档
 * @see style() - 切换样式表
 */
inline void xkcd() {
    detail::_interpreter::get();

    PyObject* res;
    PyObject *kwargs = PyDict_New();

    res = PyObject_Call(detail::_interpreter::get().s_python_function_xkcd,
            detail::_interpreter::get().s_python_empty_tuple, kwargs);

    Py_DECREF(kwargs);

    if (!res)
        throw std::runtime_error("Call to show() failed.");

    Py_DECREF(res);
}

/**
 * @brief 强制重绘当前图形
 *
 * 该函数强制Matplotlib重新绘制当前图形窗口。在交互模式下，这可以
 * 用于立即更新图形显示，而不需要等待GUI事件循环的自然刷新。
 *
 * @note 1. 通常只在交互模式下（ion() 已调用）才有明显效果。
 * @note 2. 大多数绘图函数（如 plot()、scatter()）在非交互模式下
 *         会自动触发重绘，因此此函数主要用于交互模式下的手动刷新。
 * @note 3. 等价于 Python 的 plt.draw()。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.draw() - Python官方文档
 * @see ion() - 交互模式
 * @see pause() - 暂停并刷新GUI事件循环
 */
inline void draw()
{
    detail::_interpreter::get();

    PyObject* res = PyObject_CallObject(
        detail::_interpreter::get().s_python_function_draw,
        detail::_interpreter::get().s_python_empty_tuple);

    if (!res) throw std::runtime_error("Call to draw() failed.");

    Py_DECREF(res);
}

/**
 * @brief 暂停执行并刷新GUI事件循环
 *
 * 该函数暂停程序执行指定的时间间隔（秒），同时处理GUI事件。这允许
 * 图形窗口有机会更新和响应事件（如窗口重绘、鼠标交互等）。
 *
 * @tparam Numeric 数值类型（支持 double、float、int 等可转换为 double 的类型）
 * @param interval 暂停的时间间隔（单位：秒）
 *
 * @note 1. 在交互模式下，此函数常用于创建动画效果：
 *         例如在循环中调用 plot() + pause(0.01) 实现动态绘图。
 * @note 2. 与 sleep() 不同，pause() 会处理GUI事件，保持图形窗口响应。
 * @note 3. interval=0 表示仅处理事件但不暂停（用于立即刷新）。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.pause() - Python官方文档
 * @see draw() - 仅重绘不暂停
 * @see ion() - 交互模式
 */
template<typename Numeric>
inline void pause(Numeric interval)
{
    detail::_interpreter::get();

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, PyFloat_FromDouble(interval));

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_pause, args);
    if(!res) throw std::runtime_error("Call to pause() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}

/**
 * @brief 将当前图形保存到文件
 *
 * 该函数将当前活动图形窗口的内容保存为图像文件。文件格式由文件扩展名
 * 自动确定，支持多种常见图像格式。
 *
 * @param filename 输出文件名（包括路径和扩展名）
 *                 支持的格式包括：png, pdf, ps, eps, svg, jpg, jpeg,
 *                 tif, tiff, pgf, raw, rgba 等（取决于后端）。
 *
 * @note 1. 文件路径可以是相对路径或绝对路径。相对路径相对于当前工作目录。
 * @note 2. 可以通过 dpi 关键字参数控制分辨率（默认从 rcParams 获取）。
 * @note 3. 可以通过 bbox_inches 关键字参数控制裁剪（默认 'tight'）。
 * @note 4. 可以通过 facecolor/edgecolor 控制背景色。
 * @note 5. 注意：代码中有一行调试输出 std::cout<<"args:"<<filename.c_str()<<std::endl;
 *         可能会在控制台打印文件名，影响性能或输出整洁性。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.savefig() - Python官方文档
 * @see savefig() - 同名函数别名
 */
inline void save(const std::string& filename)
{
    detail::_interpreter::get();

    PyObject* pyfilename = PyString_FromString(filename.c_str());

    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, pyfilename);
    std::cout<<"args:"<<filename.c_str()<<std::endl;

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_save, args);
    if (!res) throw std::runtime_error("Call to save() failed.");

    Py_DECREF(args);
    Py_DECREF(res);
}

/**
 * @brief 清除当前图形窗口的所有内容
 *
 * 该函数清除当前活动图形窗口中的所有坐标轴、线条、文本、图像等元素，
 * 但保持图形窗口本身打开。相当于重新初始化当前图形，可以在同一窗口
 * 中绘制新图形而不创建新窗口。
 *
 * @note 1. clf() 清除所有子图（axes），但保留图形窗口。
 * @note 2. 与 cla() 的区别：
 *          - clf() 清除整个图形（所有子图）
 *          - cla() 只清除当前活动坐标轴
 * @note 3. 清除后，图形属性（如窗口大小、背景色）保持不变。
 * @note 4. 等价于 Python 的 plt.clf()。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.clf() - Python官方文档
 * @see cla() - 清除当前坐标轴
 * @see close() - 关闭图形窗口
 */
inline void clf() {
    detail::_interpreter::get();

    PyObject *res = PyObject_CallObject(
        detail::_interpreter::get().s_python_function_clf,
        detail::_interpreter::get().s_python_empty_tuple);

    if (!res) throw std::runtime_error("Call to clf() failed.");

    Py_DECREF(res);
}

/**
 * @brief 清除当前坐标轴的所有内容
 *
 * 该函数清除当前活动坐标轴（current axes）上的所有元素（线条、文本、
 * 图像等），但保持坐标轴本身（包括坐标轴标签、刻度等框架）存在。
 * 适用于在同一个子图中重新绘制数据。
 *
 * @note 1. cla() 只影响当前活动坐标轴（gca() 返回的坐标轴）。
 * @note 2. 与 clf() 的区别：
 *          - clf() 清除整个图形（所有子图）
 *          - cla() 只清除当前坐标轴的内容
 * @note 3. 坐标轴属性（如 xlabel、ylabel、xlim、ylim）在 cla() 后
 *         会重置为默认值，除非显式重新设置。
 * @note 4. 等价于 Python 的 plt.cla()。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.cla() - Python官方文档
 * @see clf() - 清除整个图形
 * @see gca() - 获取当前坐标轴
 */
inline void cla() {
    detail::_interpreter::get();

    PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_cla,
                                        detail::_interpreter::get().s_python_empty_tuple);

    if (!res)
        throw std::runtime_error("Call to cla() failed.");

    Py_DECREF(res);
}

/**
 * @brief 启用交互模式
 *
 * 该函数启用Matplotlib的交互模式。在交互模式下，绘图命令会立即生效，
 * 图形窗口会立即更新，而不需要显式调用 show()。这适用于实时更新、
 * 动画或交互式应用程序。
 *
 * @note 1. 交互模式默认是关闭的（non-interactive）。
 * @note 2. 启用后，plot()、scatter() 等绘图函数会自动刷新图形。
 * @note 3. 在交互模式下，通常需要使用 pause() 来处理GUI事件，
 *         否则图形窗口可能无法正常响应。
 * @note 4. 使用 ioff() 可以关闭交互模式（当前C++封装未直接支持）。
 * @note 5. 等价于 Python 的 plt.ion()。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.ion() - Python官方文档
 * @see ioff() - 关闭交互模式（Python端）
 * @see show() - 显示图形（阻塞/非阻塞）
 * @see pause() - 交互模式下暂停并刷新
 */
inline void ion() {
    detail::_interpreter::get();

    PyObject *res = PyObject_CallObject(
        detail::_interpreter::get().s_python_function_ion,
        detail::_interpreter::get().s_python_empty_tuple);

    if (!res) throw std::runtime_error("Call to ion() failed.");

    Py_DECREF(res);
}

/**
 * @brief 等待用户通过鼠标在图形上点击并返回点击坐标
 *
 * 该函数阻塞程序执行，等待用户在图形窗口中点击鼠标。每次点击的坐标
 * 被记录并返回。可用于交互式数据选取、标注或测量。
 *
 * @tparam int 固定类型（函数签名指定）
 * @param numClicks 期望的点击次数，默认为 1
 * @param keywords 关键字参数字典，支持的选项包括：
 *                 - "timeout": 超时时间（秒），超时返回已点击的点
 *                 - "mouse_add": 添加点的鼠标按键（默认左键 1）
 *                 - "mouse_pop": 删除上一点的鼠标按键（默认中键 2）
 *                 - "mouse_stop": 结束点击的鼠标按键（默认右键 3）
 *
 * @return std::vector<std::array<double, 2>> 点击坐标的数组
 *         每个元素是 std::array<double, 2>，表示 (x, y) 坐标。
 *         如果用户提前关闭窗口或超时，返回已点击的点（可能少于 numClicks）。
 *
 * @note 1. 函数会阻塞直到达到指定点击次数、用户关闭窗口或超时。
 * @note 2. 返回的坐标是数据坐标系中的值，不是像素坐标。
 * @note 3. 在交互模式下，图形窗口必须已经显示（调用过 show() 或 ion()）。
 * @note 4. 如果用户点击次数不足就关闭窗口，函数会抛出异常。
 *
 * @throws std::runtime_error 当Python调用失败或用户取消时抛出异常
 *
 * @see matplotlib.pyplot.ginput() - Python官方文档
 * @see waitforbuttonpress() - 等待键盘或鼠标事件
 */
inline std::vector<std::array<double, 2>> ginput(const int numClicks = 1, const std::map<std::string, std::string>& keywords = {})
{
    detail::_interpreter::get();

    PyObject *args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, PyLong_FromLong(numClicks));

    // construct keyword args
    PyObject* kwargs = PyDict_New();
    for(std::map<std::string, std::string>::const_iterator it = keywords.begin(); it != keywords.end(); ++it)
    {
        PyDict_SetItemString(kwargs, it->first.c_str(), PyUnicode_FromString(it->second.c_str()));
    }

    PyObject* res = PyObject_Call(
        detail::_interpreter::get().s_python_function_ginput, args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(args);
    if (!res) throw std::runtime_error("Call to ginput() failed.");

    const size_t len = PyList_Size(res);
    std::vector<std::array<double, 2>> out;
    out.reserve(len);
    for (size_t i = 0; i < len; i++) {
        PyObject *current = PyList_GetItem(res, i);
        std::array<double, 2> position;
        position[0] = PyFloat_AsDouble(PyTuple_GetItem(current, 0));
        position[1] = PyFloat_AsDouble(PyTuple_GetItem(current, 1));
        out.push_back(position);
    }
    Py_DECREF(res);

    return out;
}

/**
 * @brief 自动调整子图参数，避免重叠
 *
 * 该函数自动调整当前图形的子图参数（如边距、间距），以优化布局，
 * 防止坐标轴标签、标题、刻度标签等元素相互重叠。通常在创建多个
 * 子图后调用，以确保所有元素清晰可见。
 *
 * @note 1. tight_layout() 会计算所有子图所需的空间并调整子图位置，
 *         可能会减小子图尺寸以适应标签。
 * @note 2. 可以传递参数控制padding、rect等（当前C++实现为无参版本）。
 * @note 3. 对于复杂布局（如 GridSpec），可能需要结合 constrained_layout。
 * @note 4. 建议在添加所有标题、标签、图例后调用此函数。
 * @note 5. 等价于 Python 的 plt.tight_layout()。
 *
 * @throws std::runtime_error 当Python调用失败时抛出异常
 *
 * @see matplotlib.pyplot.tight_layout() - Python官方文档
 * @see subplots_adjust() - 手动调整子图参数
 * @see constrained_layout - 另一种自动布局方案
 */
// Actually, is there any reason not to call this automatically for every plot?
inline void tight_layout() {
    detail::_interpreter::get();

    PyObject *res = PyObject_CallObject(
        detail::_interpreter::get().s_python_function_tight_layout,
        detail::_interpreter::get().s_python_empty_tuple);

    if (!res) throw std::runtime_error("Call to tight_layout() failed.");

    Py_DECREF(res);
}

// Support for variadic plot() and initializer lists:

namespace detail {

template<typename T>
using is_function = typename std::is_function<std::remove_pointer<std::remove_reference<T>>>::type;

template<bool obj, typename T>
struct is_callable_impl;

template<typename T>
struct is_callable_impl<false, T>
{
    typedef is_function<T> type;
}; // a non-object is callable iff it is a function

template<typename T>
struct is_callable_impl<true, T>
{
    struct Fallback { void operator()(); };
    struct Derived : T, Fallback { };

    template<typename U, U> struct Check;

    template<typename U>
    static std::true_type test( ... ); // use a variadic function to make sure (1) it accepts everything and (2) its always the worst match

    template<typename U>
    static std::false_type test( Check<void(Fallback::*)(), &U::operator()>* );

public:
    typedef decltype(test<Derived>(nullptr)) type;
    typedef decltype(&Fallback::operator()) dtype;
    static constexpr bool value = type::value;
}; // an object is callable iff it defines operator()

template<typename T>
struct is_callable
{
    // dispatch to is_callable_impl<true, T> or is_callable_impl<false, T> depending on whether T is of class type or not
    typedef typename is_callable_impl<std::is_class<T>::value, T>::type type;
};

template<typename IsYDataCallable>
struct plot_impl { };

/**
 * @brief plot_impl 的特化版本：处理可迭代数据（非可调用对象）
 *
 * 此结构体实现 plot_impl 的 false_type 特化，用于处理普通的可迭代容器
 * （如 std::vector、std::list、C风格数组等）。它假设第二个模板参数
 * B 是一个可迭代容器而非可调用对象，将 x 和 y 数据转换为Python列表后
 * 调用 matplotlib 的 plot 函数。
 */
template<>
struct plot_impl<std::false_type>
{
    /**
     * @brief 调用 matplotlib 的 plot 函数绘制二维折线图
     *
     * 将两个可迭代容器 x 和 y 转换为Python列表，并调用Python的
     * matplotlib.pyplot.plot() 函数绘制折线图。支持格式字符串参数。
     *
     * @tparam IterableX X数据的可迭代类型（需支持 begin/end 和 distance）
     * @tparam IterableY Y数据的可迭代类型（需支持 begin/end 和 distance）
     * @param x X轴数据容器
     * @param y Y轴数据容器
     * @param format 格式字符串，如 "r-"（红色实线）、"bo"（蓝色圆圈）等
     *
     * @return bool 成功返回 true（Python调用返回非空），失败返回 false
     *
     * @note 1. x 和 y 的元素个数必须相等，否则触发断言失败。
     * @note 2. 元素类型必须可转换为 double（通过 PyFloat_FromDouble）。
     * @note 3. format 字符串遵循MATLAB风格，可选参数可通过 keywords
     *         传递（当前重载不支持关键字，需使用其他重载）。
     *
     * @throws 无异常抛出，但Python错误会通过 PyObject_CallObject 的
     *         返回值检测，返回 false。
     *
     * @see plot_impl<std::true_type> - 处理可调用对象的版本
     */
    template<typename IterableX, typename IterableY>
    bool operator()(const IterableX& x, const IterableY& y, const std::string& format)
    {
        // 2-phase lookup for distance, begin, end
        using std::distance;
        using std::begin;
        using std::end;

        auto xs = distance(begin(x), end(x));
        auto ys = distance(begin(y), end(y));
        assert(xs == ys && "x and y data must have the same number of elements!");

        PyObject* xlist = PyList_New(xs);
        PyObject* ylist = PyList_New(ys);
        PyObject* pystring = PyString_FromString(format.c_str());

        auto itx = begin(x), ity = begin(y);
        for(size_t i = 0; i < xs; ++i) {
            PyList_SetItem(xlist, i, PyFloat_FromDouble(*itx++));
            PyList_SetItem(ylist, i, PyFloat_FromDouble(*ity++));
        }

        PyObject* plot_args = PyTuple_New(3);
        PyTuple_SetItem(plot_args, 0, xlist);
        PyTuple_SetItem(plot_args, 1, ylist);
        PyTuple_SetItem(plot_args, 2, pystring);

        PyObject* res = PyObject_CallObject(detail::_interpreter::get().s_python_function_plot, plot_args);

        Py_DECREF(plot_args);
        if(res) Py_DECREF(res);

        return res;
    }
};

/**
 * @brief plot_impl 的特化版本：处理可调用对象（函数映射）
 *
 * 此结构体实现 plot_impl 的 true_type 特化，用于处理第一种参数为
 * 可迭代容器（ticks）、第二种参数为可调用对象（函数）的情况。它计算
 * f(x) 对每个 x 的值，然后调用 false_type 版本绘制折线图。
 *
 * 这允许使用类似 Python 的语法：
 *    plot(x_values, [](double x) { return sin(x); }, "r-");
 */
template<>
struct plot_impl<std::true_type>
{
    /**
     * @brief 绘制函数曲线
     *
     * 对 ticks 中的每个 x 值计算 f(x)，收集所有 (x, f(x)) 点对后
     * 调用 plot_impl<std::false_type> 绘制折线图。
     *
     * @tparam Iterable 可迭代容器类型（x 值集合）
     * @tparam Callable 可调用对象类型（函数、函数对象、lambda等）
     * @param ticks x 值的集合（可迭代）
     * @param f 函数对象，接受一个 double 参数并返回可转换为 double 的值
     * @param format 格式字符串，传递给底层 plot 函数
     *
     * @return bool 成功返回 true，失败返回 false
     *
     * @note 1. 如果 ticks 为空容器，直接返回 true（不绘制任何内容）。
     * @note 2. 函数返回值会被隐式转换为 double 类型。
     * @note 3. 此重载支持函数式编程风格，简化函数绘图。
     *
     * @see plot_impl<std::false_type>::operator() - 实际绘图实现
     */
    template<typename Iterable, typename Callable>
    bool operator()(const Iterable& ticks, const Callable& f, const std::string& format)
    {
        if(begin(ticks) == end(ticks)) return true;

        // We could use additional meta-programming to deduce the correct element type of y,
        // but all values have to be convertible to double anyways
        std::vector<double> y;
        for(auto x : ticks) y.push_back(f(x));
        return plot_impl<std::false_type>()(ticks,y,format);
    }
};

    }
};

} // end namespace detail

/**
 * @brief 变长参数 plot() 的递归终止版本
 *
 * 这是支持可变参数模板递归的基例，当没有更多参数时停止递归。
 * 用户通常不需要直接调用此函数。
 *
 * @return bool 始终返回 true
 */
// recursion stop for the above
template<typename... Args>
bool plot() { return true; }

/**
 * @brief 变长参数 plot() 的递归处理版本
 *
 * 该函数支持一次调用绘制多条曲线，如：
 *    plot(x1, y1, "r-", x2, y2, "b--", x3, y3, "go");
 *
 * 每次递归处理一对参数，然后调用下一个 plot() 重载。
 *
 * @tparam A 第一个参数类型（通常是 x 数据容器）
 * @tparam B 第二个参数类型（通常是 y 数据容器或可调用对象）
 * @tparam Args 剩余参数包
 * @param a 第一个参数（x 数据或 ticks）
 * @param b 第二个参数（y 数据或函数）
 * @param format 格式字符串
 * @param args 其余参数对（x, y, format）或（ticks, func, format）
 *
 * @return bool 所有曲线绘制成功时返回 true，任一失败则返回 false
 *
 * @note 递归终止于 plot() 空版本。
 */
template<typename A, typename B, typename... Args>
bool plot(const A& a, const B& b, const std::string& format, Args... args)
{
    return detail::plot_impl<typename detail::is_callable<B>::type>()(a,b,format) && plot(args...);
}

/*
 * 这组 plot() 函数用于支持初始化列表（initializer lists）调用方式，
 * 例如：plot({1,2,3,4}) 或 plot({1,2}, {3,4}, "r-")。
 * 这些重载将初始化列表转换为 std::vector<double> 后调用主实现。
 */

/**
 * @brief 使用两个向量绘制折线图（显式版本）
 *
 * @param x X轴数据向量
 * @param y Y轴数据向量
 * @param format 格式字符串，默认为空（使用默认样式）
 * @return bool 绘制成功返回 true
 */
inline bool plot(const std::vector<double>& x, const std::vector<double>& y, const std::string& format = "") {
    return plot<double,double>(x,y,format);
}

/**
 * @brief 仅使用Y轴数据绘制折线图（X轴为索引）
 *
 * 当只提供Y轴数据时，X轴自动使用 0, 1, 2, ... 的索引序列。
 * 适用于简单绘制一维序列。
 *
 * @param y Y轴数据向量
 * @param format 格式字符串，默认为空
 * @return bool 绘制成功返回 true
 */
inline bool plot(const std::vector<double>& y, const std::string& format = "") {
    return plot<double>(y,format);
}

/**
 * @brief 使用两个向量和关键字参数绘制折线图
 *
 * 此重载接受关键字参数字典，可用于传递 linewidth、alpha、label 等
 * matplotlib 绘图选项。
 *
 * @param x X轴数据向量
 * @param y Y轴数据向量
 * @param keywords 关键字参数字典，键为参数名，值为字符串形式的值
 * @return bool 绘制成功返回 true
 *
 * @note 关键字参数示例：{"linewidth": "2", "alpha": "0.5", "label": "data"}
 */
inline bool plot(const std::vector<double>& x, const std::vector<double>& y, const std::map<std::string, std::string>& keywords) {
    return plot<double>(x,y,keywords);
}

/*
 * 此类支持动态绘图，即在不清除和重新绘图的情况下更改已绘制的数据。
 * 它封装了 matplotlib 的 Line2D 对象，提供 update() 方法实时更新
 * 数据，适用于动画、实时数据流等场景。
 *
 * 使用示例：
 *   Plot p("myline", x, y, "r-");  // 创建动态折线
 *   // ... 后续更新 ...
 *   p.update(new_x, new_y);        // 更新数据
 *   p.clear();                     // 清空数据但保留对象
 *   p.remove();                    // 从图形中移除该线条
 */
class Plot
{
public:
    /**
     * @brief 构造函数：创建动态折线图对象并初始绘制
     *
     * 使用提供的名称（标签）、X/Y数据和格式字符串创建一个 Plot 对象。
     * 构造函数内部调用 matplotlib 的 plot() 函数创建线条，并保存
     * set_data 方法指针用于后续更新。
     *
     * @tparam Numeric 数值类型（支持 double、float、int 等）
     * @param name 图例标签名称，空字符串表示不设置标签
     * @param x X轴数据向量
     * @param y Y轴数据向量
     * @param format 格式字符串，如 "r-"（红色实线）、"b:"（蓝色点线）等
     *
     * @note 1. x 和 y 的大小必须相等，否则触发断言失败。
     * @note 2. 如果 format 为空，使用 matplotlib 默认样式。
     * @note 3. name 不为空时，会设置 label 属性，后续需要调用 legend()
     *         才能显示图例。
     * @note 4. 构造成功后，对象持有对Python Line2D 对象的引用，需要
     *         注意生命周期管理（析构时会自动释放）。
     *
     * @see update() - 更新数据
     * @see clear() - 清空数据
     * @see remove() - 移除线条
     */
    template<typename Numeric>
    Plot(const std::string& name, const std::vector<Numeric>& x, const std::vector<Numeric>& y, const std::string& format = "") {
        detail::_interpreter::get();

        assert(x.size() == y.size());

        PyObject* kwargs = PyDict_New();
        if(name != "")
            PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

        PyObject* xarray = detail::get_array(x);
        PyObject* yarray = detail::get_array(y);

        PyObject* pystring = PyString_FromString(format.c_str());

        PyObject* plot_args = PyTuple_New(3);
        PyTuple_SetItem(plot_args, 0, xarray);
        PyTuple_SetItem(plot_args, 1, yarray);
        PyTuple_SetItem(plot_args, 2, pystring);

        PyObject* res = PyObject_Call(detail::_interpreter::get().s_python_function_plot, plot_args, kwargs);

        Py_DECREF(kwargs);
        Py_DECREF(plot_args);

        if(res)
        {
            line= PyList_GetItem(res, 0);

            if(line)
                set_data_fct = PyObject_GetAttrString(line,"set_data");
            else
                Py_DECREF(line);
            Py_DECREF(res);
        }
    }

    /**
     * @brief 简化构造函数：创建空的数据容器，稍后填充
     *
     * 此构造函数调用第一个构造函数的委托构造函数，创建空的 x 和 y
     * 向量。适用于需要先创建 Plot 对象，后续再调用 update() 填充
     * 数据的场景（如实时数据流）。
     *
     * @param name 图例标签名称，默认为空字符串
     * @param format 格式字符串，默认为空
     *
     * @note 构造后线条为空，不会在图形中显示任何数据点，直到首次
     *       调用 update() 提供有效数据。
     */
    // shorter initialization with name or format only
    // basically calls line, = plot([], [])
    Plot(const std::string& name = "", const std::string& format = "")
        : Plot(name, std::vector<double>(), std::vector<double>(), format) {}

    /**
     * @brief 更新已绘制线条的数据
     *
     * 调用 Line2D.set_data() 方法更新线条的 X 和 Y 数据。图形会自动
     * 刷新显示更新后的数据（在交互模式下立即生效，非交互模式下需要
     * 调用 draw() 或 pause() 刷新）。
     *
     * @tparam Numeric 数值类型（可转换为 double）
     * @param x 新的X轴数据向量
     * @param y 新的Y轴数据向量
     *
     * @return bool 更新成功返回 true，失败返回 false（如对象已销毁）
     *
     * @note 1. x 和 y 的大小必须相等。
     * @note 2. 此方法不会改变线条的其他属性（颜色、线型、标签等）。
     * @note 3. 多次调用 update() 可实现动画效果。
     * @note 4. 在非交互模式下，更新后可能需要调用 draw() 才能看到变化。
     *
     * @see clear() - 清空数据（调用 update(空向量, 空向量)）
     * @see Plot() - 构造函数
     */
    template<typename Numeric>
    bool update(const std::vector<Numeric>& x, const std::vector<Numeric>& y) {
        assert(x.size() == y.size());
        if(set_data_fct)
        {
            PyObject* xarray = detail::get_array(x);
            PyObject* yarray = detail::get_array(y);

            PyObject* plot_args = PyTuple_New(2);
            PyTuple_SetItem(plot_args, 0, xarray);
            PyTuple_SetItem(plot_args, 1, yarray);

            PyObject* res = PyObject_CallObject(set_data_fct, plot_args);
            if (res) Py_DECREF(res);
            return res;
        }
        return false;
    }

    /**
     * @brief 清空线条数据但不移除线条对象
     *
     * 调用 update() 传入空向量，清空线条的数据点。线条对象仍然存在
     * 于图形中（但不可见），可以后续再次调用 update() 填充新数据。
     *
     * @return bool 成功返回 true，失败返回 false
     *
     * @note 此方法适用于需要重置线条状态的场景，比 remove() + 重建
     *       更高效，因为保留了线条属性和引用。
     *
     * @see update() - 更新数据
     * @see remove() - 完全移除线条
     * @see clear() - 同名方法
     */
    // clears the plot but keep it available
    bool clear() {
        return update(std::vector<double>(), std::vector<double>());
    }

    /**
     * @brief 从图形中完全移除线条
     *
     * 调用 Line2D.remove() 方法将线条从坐标轴中移除，并释放相关资源。
     * 移除后，Plot 对象不再有效，不应再调用 update() 等方法。
     *
     * @note 1. remove() 后对象处于无效状态，析构时不会重复释放。
     * @note 2. 与 clear() 的区别：clear() 仅清空数据但保留对象，remove()
     *         完全删除对象。
     * @note 3. 移除后如需重新绘制，需要创建新的 Plot 对象。
     *
     * @see clear() - 清空数据但保留对象
     * @see ~Plot() - 析构函数（自动调用 decref 释放资源）
     */
    // definitely remove this line
    void remove() {
        if(line)
        {
            auto remove_fct = PyObject_GetAttrString(line,"remove");
            PyObject* args = PyTuple_New(0);
            PyObject* res = PyObject_CallObject(remove_fct, args);
            if (res) Py_DECREF(res);
        }
        decref();
    }

    /**
     * @brief 析构函数
     *
     * 自动调用 decref() 释放持有的 Python 对象引用（line 和 set_data_fct）。
     * 确保 Plot 对象生命周期结束时正确释放资源，避免Python引用计数泄漏。
     *
     * @note C++对象销毁时，Python对象引用计数会递减，可能触发Python垃圾回收。
     */
    ~Plot() {
        decref();
    }
private:

    /**
     * @brief 释放Python对象引用
     *
     * 递减 line 和 set_data_fct 两个PyObject*的引用计数。
     * 如果引用计数降至0，Python将释放对应的对象。
     *
     * @note 1. 此函数在析构函数和 remove() 中被调用。
     * @note 2. 使用 Py_DECREF 宏递减引用计数，这是Python C API的标准做法。
     * @note 3. 调用前会检查指针是否为 nullptr，避免重复释放。
     */
    void decref() {
        if(line)
            Py_DECREF(line);
        if(set_data_fct)
            Py_DECREF(set_data_fct);
    }


    /**
     * @brief 指向Python Line2D对象的指针
     *
     * 存储 matplotlib.lines.Line2D 实例的 PyObject* 引用。
     * 该对象代表图形中的一条折线，通过 set_data 方法更新数据。
     */
    PyObject* line = nullptr;

    /**
     * @brief 指向Line2D.set_data方法的指针
     *
     * 存储 line 对象的 set_data 方法的 PyObject* 引用。
     * update() 方法调用此函数指针来更新线条数据，避免每次更新
     * 都重新查找方法，提高性能。
     */
    PyObject* set_data_fct = nullptr;
};

} // end namespace matplotlibcpp
