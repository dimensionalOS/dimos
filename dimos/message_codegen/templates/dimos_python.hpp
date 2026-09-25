// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <vector>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace dimos::python {
namespace py = pybind11;

struct BorrowState {
    size_t count = 0;
};

inline py::object owner_root(py::object owner) {
    return py::hasattr(owner, "__dimos_owner") ? owner.attr("__dimos_owner") : owner;
}

inline std::shared_ptr<BorrowState> borrow_state(py::object owner) {
    owner = owner_root(owner);
    if (!py::hasattr(owner, "__dimos_buffers")) {
        owner.attr("__dimos_buffers") = py::capsule(
            new std::shared_ptr<BorrowState>(std::make_shared<BorrowState>()),
            [](void* value) { delete static_cast<std::shared_ptr<BorrowState>*>(value); });
    }
    auto capsule = owner.attr("__dimos_buffers").cast<py::capsule>();
    return *static_cast<std::shared_ptr<BorrowState>*>(capsule.get_pointer());
}

inline void require_unborrowed(py::object owner) {
    if (borrow_state(owner)->count != 0) {
        throw py::buffer_error("Cannot replace or resize message storage while a NumPy view is borrowed");
    }
}

struct Borrow {
    py::object owner;
    std::shared_ptr<BorrowState> state;

    explicit Borrow(py::object value) : owner(owner_root(value)), state(borrow_state(owner)) {
        ++state->count;
    }
    ~Borrow() { --state->count; }
};

template<class Container>
struct Sequence {
    Container* values;
    py::object owner;
};

template<class Container>
Container sequence_from_python(py::handle value) {
    using T = typename Container::value_type;
    if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
        if (PyObject_CheckBuffer(value.ptr())) {
            auto info = py::reinterpret_borrow<py::buffer>(value).request();
            if (info.ndim == 1 && info.itemsize == sizeof(T) &&
                info.format == py::format_descriptor<T>::format() && info.strides[0] == sizeof(T)) {
                const auto* first = static_cast<const T*>(info.ptr);
                if constexpr (std::is_same_v<Container, std::vector<T>>) {
                    return Container(first, first + info.size);
                } else {
                    Container result{};
                    if (static_cast<size_t>(info.size) != result.size()) {
                        throw py::value_error("Incorrect fixed-array length");
                    }
                    std::copy(first, first + info.size, result.begin());
                    return result;
                }
            }
        }
    }
    return py::cast<Container>(value);
}

template<class Container>
void bind_sequence(py::module_& module, const char* name) {
    if (py::hasattr(module, name)) return;
    using T = typename Container::value_type;
    using View = Sequence<Container>;
    auto cls = py::class_<View>(module, name, py::module_local());
    cls.def("__len__", [](const View& self) { return self.values->size(); });
    cls.def("__getitem__", [](const View& self, py::ssize_t index) {
        if (index < 0) index += self.values->size();
        if (index < 0 || static_cast<size_t>(index) >= self.values->size()) throw py::index_error();
        // Message elements are values. Assign an edited element back explicitly;
        // a vector resize must never leave a Python object pointing into it.
        return T((*self.values)[index]);
    });
    cls.def("__setitem__", [](View& self, py::ssize_t index, const T& value) {
        if (index < 0) index += self.values->size();
        if (index < 0 || static_cast<size_t>(index) >= self.values->size()) throw py::index_error();
        (*self.values)[index] = value;
    });
    cls.def("__iter__", [](const View& self) {
        return py::iter(py::cast(Container(*self.values)));
    });
    cls.def("__repr__", [](const View& self) { return py::repr(py::cast(*self.values)); });
    cls.def("__eq__", [](const View& self, py::object other) {
        if (!PySequence_Check(other.ptr())) return false;
        return py::cast(*self.values).equal(py::list(other));
    });
    if constexpr (std::is_same_v<Container, std::vector<T>>) {
        cls.def("append", [](View& self, const T& value) {
            require_unborrowed(self.owner);
            self.values->push_back(value);
        });
        cls.def("extend", [](View& self, py::iterable values) {
            require_unborrowed(self.owner);
            auto extra = sequence_from_python<Container>(values);
            self.values->insert(self.values->end(), extra.begin(), extra.end());
        });
        cls.def("clear", [](View& self) {
            require_unborrowed(self.owner);
            self.values->clear();
        });
    }
    if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
        cls.def("view", [](const View& self) {
            py::capsule owner(new Borrow(self.owner), [](void* value) { delete static_cast<Borrow*>(value); });
            py::array result(py::dtype::of<T>(), {self.values->size()}, {sizeof(T)}, self.values->data(), owner);
            result.attr("setflags")(py::arg("write") = false);
            return result;
        });
        cls.def("copy", [](const View& self) {
            py::array_t<T> result(self.values->size());
            std::copy(self.values->begin(), self.values->end(), result.mutable_data());
            return result;
        });
    }
}

}  // namespace dimos::python
