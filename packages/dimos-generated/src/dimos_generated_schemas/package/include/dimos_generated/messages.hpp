// Generated from ROS2 .msg definitions. Do not edit.
#pragma once
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <fastcdr/Cdr.h>
#include <fastcdr/CdrSizeCalculator.hpp>
#include "dimos_cdr.hpp"
// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_RUNTIME_C__MESSAGE_INITIALIZATION_H_
#define ROSIDL_RUNTIME_C__MESSAGE_INITIALIZATION_H_

enum rosidl_runtime_c__message_initialization
{
  // Initialize all fields of the message, either with the default value
  // (if the field has one), or with an empty value (generally 0 or an
  // empty string).
  ROSIDL_RUNTIME_C_MSG_INIT_ALL,
  // Skip initialization of all fields of the message.  It is up to the user to
  // ensure that all fields are initialized before use.
  ROSIDL_RUNTIME_C_MSG_INIT_SKIP,
  // Initialize all fields of the message to an empty value (generally 0 or an
  // empty string).
  ROSIDL_RUNTIME_C_MSG_INIT_ZERO,
  // Initialize all fields of the message that have defaults; all other fields
  // are left untouched.
  ROSIDL_RUNTIME_C_MSG_INIT_DEFAULTS_ONLY,
};

#endif  // ROSIDL_RUNTIME_C__MESSAGE_INITIALIZATION_H_
// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_RUNTIME_CPP__MESSAGE_INITIALIZATION_HPP_
#define ROSIDL_RUNTIME_CPP__MESSAGE_INITIALIZATION_HPP_


namespace rosidl_runtime_cpp
{

/// Enum utilized in rosidl generated sources for describing how members are initialized.
/**
 * See the documentation for the `rosidl_runtime_c__message_initialization` enum for more information.
 */
enum class MessageInitialization
{
  ALL = ROSIDL_RUNTIME_C_MSG_INIT_ALL,
  SKIP = ROSIDL_RUNTIME_C_MSG_INIT_SKIP,
  ZERO = ROSIDL_RUNTIME_C_MSG_INIT_ZERO,
  DEFAULTS_ONLY = ROSIDL_RUNTIME_C_MSG_INIT_DEFAULTS_ONLY,
};

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__MESSAGE_INITIALIZATION_HPP_
// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_RUNTIME_CPP__BOUNDED_VECTOR_HPP_
#define ROSIDL_RUNTIME_CPP__BOUNDED_VECTOR_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

// GCC 13 has false positive warnings around stringop-overflow and array-bounds.
// The layout of a BoundedVector<bool> triggers these warnings.  Suppress them
// until this is fixed in upstream gcc.  See
// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=114758 for more details.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif

namespace rosidl_runtime_cpp
{

/// A container based on std::vector but with an upper bound.
/**
 * Meets the same requirements as std::vector.
 *
 * \param Tp Type of element
 * \param UpperBound The upper bound for the number of elements
 * \param Alloc Allocator type, defaults to std::allocator<Tp>
 */
template<typename Tp, std::size_t UpperBound, typename Alloc = std::allocator<Tp>>
class BoundedVector
  : protected std::vector<Tp, Alloc>
{
  using Base = std::vector<Tp, Alloc>;

public:
  using typename Base::value_type;
  using typename Base::pointer;
  using typename Base::const_pointer;
  using typename Base::reference;
  using typename Base::const_reference;
  using typename Base::iterator;
  using typename Base::const_iterator;
  using typename Base::const_reverse_iterator;
  using typename Base::reverse_iterator;
  using typename Base::size_type;
  using typename Base::difference_type;
  using typename Base::allocator_type;

  /// Create a %BoundedVector with no elements.
  BoundedVector()
  noexcept (std::is_nothrow_default_constructible<Alloc>::value)
  : Base()
  {}

  /// Creates a %BoundedVector with no elements.
  /**
   * \param a An allocator object
   */
  explicit
  BoundedVector(
    const typename Base::allocator_type & a)
  noexcept
  : Base(a)
  {}

  /// Create a %BoundedVector with default constructed elements.
  /**
   * This constructor fills the %BoundedVector with @a n default
   * constructed elements.
   *
   * \param n The number of elements to initially create
   * \param a An allocator
   */
  explicit
  BoundedVector(
    typename Base::size_type n,
    const typename Base::allocator_type & a = allocator_type())
  : Base(n, a)
  {
    if (n > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
  }

  /// Create a %BoundedVector with copies of an exemplar element.
  /**
   * This constructor fills the %BoundedVector with @a n copies of @a value.
   *
   * \param n The number of elements to initially create
   * \param value An element to copy
   * \param a An allocator
   */
  BoundedVector(
    typename Base::size_type n,
    const typename Base::value_type & value,
    const typename Base::allocator_type & a = allocator_type())
  : Base(n, value, a)
  {
    if (n > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
  }

  /// %BoundedVector copy constructor.
  /**
   * The newly-created %BoundedVector uses a copy of the allocation
   * object used by @a x.
   * All the elements of @a x are copied, but any extra memory in
   * @a x (for fast expansion) will not be copied.
   *
   * \param x A %BoundedVector of identical element and allocator types
   */
  BoundedVector(
    const BoundedVector & x)
  : Base(x)
  {}

  /// %BoundedVector move constructor.
  /**
   * The newly-created %BoundedVector contains the exact contents of @a x.
   * The contents of @a x are a valid, but unspecified %BoundedVector.
   *
   * \param x A %BoundedVector of identical element and allocator types
   */
  BoundedVector(BoundedVector && x) noexcept
  : Base(std::move(x))
  {}

  /// Copy constructor with alternative allocator
  BoundedVector(const BoundedVector & x, const typename Base::allocator_type & a)
  : Base(x, a)
  {}

  /// Build a %BoundedVector from an initializer list.
  /**
   * Create a %BoundedVector consisting of copies of the elements in the
   * initializer_list @a l.
   *
   * This will call the element type's copy constructor N times
   * (where N is @a l.size()) and do no memory reallocation.
   *
   * \param l An initializer_list
   * \param a An allocator
   */
  BoundedVector(
    std::initializer_list<typename Base::value_type> l,
    const typename Base::allocator_type & a = typename Base::allocator_type())
  : Base(l, a)
  {
    if (l.size() > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
  }

  /// Build a %BoundedVector from a range.
  /**
   * Create a %BoundedVector consisting of copies of the elements from
   * [first,last).
   *
   * If the iterators are forward, bidirectional, or random-access, then
   * this will call the elements' copy constructor N times (where N is
   * distance(first,last)) and do no memory reallocation.
   * But if only input iterators are used, then this will do at most 2N
   * calls to the copy constructor, and logN memory reallocations.
   *
   * \param first An input iterator
   * \param last An input iterator
   * \param a An allocator
   */
  template<
    typename InputIterator
  >
  BoundedVector(
    InputIterator first,
    InputIterator last,
    const typename Base::allocator_type & a = allocator_type())
  : Base(first, last, a)
  {
    if (size() > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
  }

  /// The dtor only erases the elements.
  /**
   * Note that if the elements themselves are pointers, the pointed-to
   * memory is not touched in any way.
   * Managing the pointer is the user's responsibility.
   */
  ~BoundedVector() noexcept
  {}

  /// %BoundedVector assignment operator.
  /**
   * All the elements of @a x are copied, but any extra memory in
   * @a x (for fast expansion) will not be copied.
   * Unlike the copy constructor, the allocator object is not copied.
   *
   * \param x A %BoundedVector of identical element and allocator types
   */
  BoundedVector &
  operator=(const BoundedVector & x)
  {
    (void)Base::operator=(x);
    return *this;
  }

  /// %BoundedVector move assignment operator
  /**
   * \param x A %BoundedVector of identical element and allocator types.
   */
  BoundedVector &
  operator=(BoundedVector && x)
  {
    (void)Base::operator=(std::move(x));
    return *this;
  }

  /// %BoundedVector list assignment operator.
  /**
   * This function fills a %BoundedVector with copies of the elements in
   * the initializer list @a l.
   *
   * Note that the assignment completely changes the %BoundedVector and
   * that the resulting %BoundedVector's size is the same as the number
   * of elements assigned.
   * Old data may be lost.
   *
   * \param l An initializer_list
   */
  BoundedVector &
  operator=(std::initializer_list<typename Base::value_type> l)
  {
    if (l.size() > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::operator=(l);
    return *this;
  }

  /// Assign a given value to a %BoundedVector.
  /**
   * This function fills a %BoundedVector with @a n copies of the
   * given value.
   * Note that the assignment completely changes the %BoundedVector and
   * that the resulting %BoundedVector's size is the same as the number
   * of elements assigned.
   * Old data may be lost.
   *
   * \param n Number of elements to be assigned
   * \param val Value to be assigned
   */
  void
  assign(
    typename Base::size_type n,
    const typename Base::value_type & val)
  {
    if (n > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::assign(n, val);
  }

  /// Assign a range to a %BoundedVector.
  /**
   * This function fills a %BoundedVector with copies of the elements in
   * the range [first,last).
   *
   * Note that the assignment completely changes the %BoundedVector and
   * that the resulting %BoundedVector's size is the same as the number
   * of elements assigned.
   * Old data may be lost.
   *
   * \param first An input iterator
   * \param last   An input iterator
   */
  template<
    typename InputIterator
  >
  void
  assign(InputIterator first, InputIterator last)
  {
    using cat = typename std::iterator_traits<InputIterator>::iterator_category;
    do_assign(first, last, cat());
  }

  /// Assign an initializer list to a %BoundedVector.
  /**
   * This function fills a %BoundedVector with copies of the elements in
   * the initializer list @a l.
   *
   * Note that the assignment completely changes the %BoundedVector and
   * that the resulting %BoundedVector's size is the same as the number
   * of elements assigned.
   * Old data may be lost.
   *
   * \param l An initializer_list
   */
  void
  assign(std::initializer_list<typename Base::value_type> l)
  {
    if (l.size() > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::assign(l);
  }

  using Base::begin;
  using Base::end;
  using Base::rbegin;
  using Base::rend;
  using Base::cbegin;
  using Base::cend;
  using Base::crbegin;
  using Base::crend;
  using Base::size;

  /** Returns the size() of the largest possible %BoundedVector.  */
  typename Base::size_type
  max_size() const noexcept
  {
    return std::min(UpperBound, Base::max_size());
  }

  /// Resize the %BoundedVector to the specified number of elements.
  /**
   * This function will %resize the %BoundedVector to the specified
   * number of elements.
   * If the number is smaller than the %BoundedVector's current size the
   * %BoundedVector is truncated, otherwise default constructed elements
   * are appended.
   *
   * \param new_size Number of elements the %BoundedVector should contain
   */
  void
  resize(typename Base::size_type new_size)
  {
    if (new_size > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::resize(new_size);
  }

  /// Resize the %BoundedVector to the specified number of elements.
  /**
   * This function will %resize the %BoundedVector to the specified
   * number of elements.
   * If the number is smaller than the %BoundedVector's current size the
   * %BoundedVector is truncated, otherwise the %BoundedVector is
   * extended and new elements are populated with given data.
   *
   * \param new_size Number of elements the %BoundedVector should contain
   * \param x Data with which new elements should be populated
   */
  void
  resize(
    typename Base::size_type new_size,
    const typename Base::value_type & x)
  {
    if (new_size > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::resize(new_size, x);
  }

  using Base::shrink_to_fit;
  using Base::capacity;
  using Base::empty;

  /// Attempt to preallocate enough memory for specified number of elements.
  /**
   * This function attempts to reserve enough memory for the
   * %BoundedVector to hold the specified number of elements.
   * If the number requested is more than max_size(), length_error is
   * thrown.
   *
   * The advantage of this function is that if optimal code is a
   * necessity and the user can determine the number of elements that
   * will be required, the user can reserve the memory in %advance, and
   * thus prevent a possible reallocation of memory and copying of
   * %BoundedVector data.
   *
   * \param n Number of elements required
   * @throw std::length_error If @a n exceeds @c max_size()
   */
  void
  reserve(typename Base::size_type n)
  {
    if (n > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::reserve(n);
  }

  using Base::operator[];
  using Base::at;
  using Base::front;
  using Base::back;

  /// Return a pointer such that [data(), data() + size()) is a valid range.
  /**
   * For a non-empty %BoundedVector, data() == &front().
   */
  template<
    typename T,
    typename std::enable_if<
      !std::is_same<T, Tp>::value &&
      !std::is_same<T, bool>::value
    >::type * = nullptr
  >
  T *
  data() noexcept
  {
    return Base::data();
  }

  template<
    typename T,
    typename std::enable_if<
      !std::is_same<T, Tp>::value &&
      !std::is_same<T, bool>::value
    >::type * = nullptr
  >
  const T *
  data() const noexcept
  {
    return Base::data();
  }

  /// Add data to the end of the %BoundedVector.
  /**
   * This is a typical stack operation.
   * The function creates an element at the end of the %BoundedVector
   * and assigns the given data to it.
   * Due to the nature of a %BoundedVector this operation can be done in
   * constant time if the %BoundedVector has preallocated space
   * available.
   *
   * \param x Data to be added
   */
  void
  push_back(const typename Base::value_type & x)
  {
    if (size() >= UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::push_back(x);
  }

  void
  push_back(typename Base::value_type && x)
  {
    if (size() >= UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::push_back(x);
  }

  /// Add data to the end of the %BoundedVector.
  /**
   * This is a typical stack operation.
   * The function creates an element at the end of the %BoundedVector
   * and assigns the given data to it.
   * Due to the nature of a %BoundedVector this operation can be done in
   * constant time if the %BoundedVector has preallocated space
   * available.
   *
   * \param args Arguments to be forwarded to the constructor of Tp
   */
  template<typename ... Args>
  typename Base::reference
  emplace_back(Args && ... args)
  {
    if (size() >= UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return Base::emplace_back(std::forward<Args>(args)...);
  }

  /// Insert an object in %BoundedVector before specified iterator.
  /**
   * This function will insert an object of type T constructed with
   * T(std::forward<Args>(args)...) before the specified location.
   * Note that this kind of operation could be expensive for a
   * %BoundedVector and if it is frequently used the user should
   * consider using std::list.
   *
   * \param position A const_iterator into the %BoundedVector
   * \param args Arguments
   * \return An iterator that points to the inserted data
   */
  template<typename ... Args>
  typename Base::iterator
  emplace(
    typename Base::const_iterator position,
    Args && ... args)
  {
    if (size() >= UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return Base::emplace(position, std::forward<Args>(args) ...);
  }

  /// Insert given value into %BoundedVector before specified iterator.
  /**
   * This function will insert a copy of the given value before the
   * specified location.
   * Note that this kind of operation could be expensive for a
   * %BoundedVector and if it is frequently used the user should
   * consider using std::list.
   *
   * \param position A const_iterator into the %BoundedVector
   * \param x Data to be inserted
   * \return An iterator that points to the inserted data
   */
  typename Base::iterator
  insert(
    typename Base::const_iterator position,
    const typename Base::value_type & x)
  {
    if (size() >= UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return Base::insert(position, x);
  }

  /// Insert given rvalue into %BoundedVector before specified iterator.
  /**
   * This function will insert a copy of the given rvalue before the
   * specified location.
   * Note that this kind of operation could be expensive for a
   * %BoundedVector and if it is frequently used the user should
   * consider using std::list.
   *
   * \param position A const_iterator into the %BoundedVector
   * \param x Data to be inserted
   * \return An iterator that points to the inserted data
   */
  typename Base::iterator
  insert(
    typename Base::const_iterator position,
    typename Base::value_type && x)
  {
    if (size() >= UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return Base::insert(position, x);
  }

  /// Insert an initializer_list into the %BoundedVector.
  /**
   * This function will insert copies of the data in the
   * initializer_list @a l into the %BoundedVector before the location
   * specified by @a position.
   *
   * Note that this kind of operation could be expensive for a
   * %BoundedVector and if it is frequently used the user should
   * consider using std::list.
   *
   * \param position An iterator into the %BoundedVector
   * \param l An initializer_list
   */
  typename Base::iterator
  insert(
    typename Base::const_iterator position,
    std::initializer_list<typename Base::value_type> l)
  {
    if (size() + l.size() > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return Base::insert(position, l);
  }

  /// Insert a number of copies of given data into the %BoundedVector.
  /**
   * This function will insert a specified number of copies of the given
   * data before the location specified by @a position.
   *
   * Note that this kind of operation could be expensive for a
   * %BoundedVector and if it is frequently used the user should
   * consider using std::list.
   *
   * \param position A const_iterator into the %BoundedVector
   * \param n Number of elements to be inserted
   * \param x Data to be inserted
   * \return An iterator that points to the inserted data
   */
  typename Base::iterator
  insert(
    typename Base::const_iterator position,
    typename Base::size_type n,
    const typename Base::value_type & x)
  {
    if (size() + n > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    return Base::insert(position, n, x);
  }

  /// Insert a range into the %BoundedVector.
  /**
   * This function will insert copies of the data in the range
   * [first,last) into the %BoundedVector before the location
   * specified by @a pos.
   *
   * Note that this kind of operation could be expensive for a
   * %BoundedVector and if it is frequently used the user should
   * consider using std::list.
   *
   * \param position A const_iterator into the %BoundedVector
   * \param first An input iterator
   * \param last   An input iterator
   * \return An iterator that points to the inserted data
   */
  template<
    typename InputIterator
  >
  typename Base::iterator
  insert(
    typename Base::const_iterator position,
    InputIterator first,
    InputIterator last)
  {
    using cat = typename std::iterator_traits<InputIterator>::iterator_category;
    return do_insert(position, first, last, cat());
  }

  using Base::erase;
  using Base::pop_back;
  using Base::clear;

private:
  /// Assign elements from an input range.
  template<
    typename InputIterator
  >
  void
  do_assign(InputIterator first, InputIterator last, std::input_iterator_tag)
  {
    BoundedVector(first, last).swap(*this);
  }

  /// Assign elements from a forward range.
  template<
    typename FwdIterator
  >
  void
  do_assign(FwdIterator first, FwdIterator last, std::forward_iterator_tag)
  {
    if (static_cast<std::size_t>(std::distance(first, last)) > UpperBound) {
      throw std::length_error("Exceeded upper bound");
    }
    Base::assign(first, last);
  }

  // Insert each value at the end and then rotate them to the desired position.
  // If the bound is exceeded, the inserted elements are removed again.
  template<
    typename InputIterator
  >
  typename Base::iterator
  do_insert(
    typename Base::const_iterator position,
    InputIterator first,
    InputIterator last,
    std::input_iterator_tag)
  {
    const auto orig_size = size();
    const auto idx = position - cbegin();
    try {
      while (first != last) {
        push_back(*first++);
      }
    } catch (const std::length_error &) {
      Base::resize(orig_size);
      throw;
    }
    auto pos = begin() + idx;
    std::rotate(pos, begin() + orig_size, end());
    return begin() + idx;
  }

  template<
    typename FwdIterator
  >
  typename Base::iterator
  do_insert(
    typename Base::const_iterator position,
    FwdIterator first,
    FwdIterator last,
    std::forward_iterator_tag)
  {
    auto dist = std::distance(first, last);
    if ((dist < 0) || (size() + static_cast<size_t>(dist) > UpperBound)) {
      throw std::length_error("Exceeded upper bound");
    }
    return Base::insert(position, first, last);
  }

  /// Vector equality comparison.
  /**
   * This is an equivalence relation.
   * It is linear in the size of the vectors.
   * Vectors are considered equivalent if their sizes are equal, and if
   * corresponding elements compare equal.
   *
   * \param x A %BoundedVector
   * \param y A %BoundedVector of the same type as @a x
   * \return True if the size and elements of the vectors are equal
  */
  friend bool
  operator==(
    const BoundedVector & x,
    const BoundedVector & y)
  {
    return static_cast<const Base &>(x) == static_cast<const Base &>(y);
  }

  /// Vector ordering relation.
  /**
   * This is a total ordering relation.
   * It is linear in the size of the vectors.
   * The elements must be comparable with @c <.
   *
   * See std::lexicographical_compare() for how the determination is made.
   *
   * \param x A %BoundedVector
   * \param y A %BoundedVector of the same type as @a x
   * @return True if @a x is lexicographically less than @a y
  */
  friend bool
  operator<(
    const BoundedVector & x,
    const BoundedVector & y)
  {
    return static_cast<const Base &>(x) < static_cast<const Base &>(y);
  }

  /// Based on operator==
  friend bool
  operator!=(
    const BoundedVector & x,
    const BoundedVector & y)
  {
    return static_cast<const Base &>(x) != static_cast<const Base &>(y);
  }

  /// Based on operator<
  friend bool
  operator>(
    const BoundedVector & x,
    const BoundedVector & y)
  {
    return static_cast<const Base &>(x) > static_cast<const Base &>(y);
  }

  /// Based on operator<
  friend bool
  operator<=(
    const BoundedVector & x,
    const BoundedVector & y)
  {
    return static_cast<const Base &>(x) <= static_cast<const Base &>(y);
  }

  /// Based on operator<
  friend bool
  operator>=(
    const BoundedVector & x,
    const BoundedVector & y)
  {
    return static_cast<const Base &>(x) >= static_cast<const Base &>(y);
  }
};

/// See rosidl_runtime_cpp::BoundedVector::swap().
template<typename Tp, std::size_t UpperBound, typename Alloc>
inline void
swap(BoundedVector<Tp, UpperBound, Alloc> & x, BoundedVector<Tp, UpperBound, Alloc> & y)
{
  x.swap(y);
}

}  // namespace rosidl_runtime_cpp

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

#endif  // ROSIDL_RUNTIME_CPP__BOUNDED_VECTOR_HPP_
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from builtin_interfaces:msg/Duration.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "builtin_interfaces/msg/duration.hpp"


#ifndef DIMOS_CDR_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C
#define DIMOS_CDR_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__builtin_interfaces__msg__Duration __attribute__((deprecated))
#else
# define DEPRECATED__builtin_interfaces__msg__Duration __declspec(deprecated)
#endif

namespace builtin_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Duration_
{
  using Type = Duration_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "builtin_interfaces/msg/Duration";

  explicit Duration_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sec = 0l;
      this->nanosec = 0ul;
    }
  }

  explicit Duration_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sec = 0l;
      this->nanosec = 0ul;
    }
  }

  // field types and members
  using _sec_type =
    int32_t;
  _sec_type sec;
  using _nanosec_type =
    uint32_t;
  _nanosec_type nanosec;

  // setters for named parameter idiom
  Type & set__sec(
    const int32_t & _arg)
  {
    this->sec = _arg;
    return *this;
  }
  Type & set__nanosec(
    const uint32_t & _arg)
  {
    this->nanosec = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    builtin_interfaces::msg::Duration_<ContainerAllocator> *;
  using ConstRawPtr =
    const builtin_interfaces::msg::Duration_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<builtin_interfaces::msg::Duration_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<builtin_interfaces::msg::Duration_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      builtin_interfaces::msg::Duration_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<builtin_interfaces::msg::Duration_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      builtin_interfaces::msg::Duration_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<builtin_interfaces::msg::Duration_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<builtin_interfaces::msg::Duration_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<builtin_interfaces::msg::Duration_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__builtin_interfaces__msg__Duration
    std::shared_ptr<builtin_interfaces::msg::Duration_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__builtin_interfaces__msg__Duration
    std::shared_ptr<builtin_interfaces::msg::Duration_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Duration_ & other) const
  {
    if (this->sec != other.sec) {
      return false;
    }
    if (this->nanosec != other.nanosec) {
      return false;
    }
    return true;
  }
  bool operator!=(const Duration_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Duration_

// alias to use template instance with default allocator
using Duration =
  builtin_interfaces::msg::Duration_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace builtin_interfaces

#endif  // DIMOS_CDR_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from builtin_interfaces:msg/Time.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "builtin_interfaces/msg/time.hpp"


#ifndef DIMOS_CDR_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4
#define DIMOS_CDR_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__builtin_interfaces__msg__Time __attribute__((deprecated))
#else
# define DEPRECATED__builtin_interfaces__msg__Time __declspec(deprecated)
#endif

namespace builtin_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Time_
{
  using Type = Time_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "builtin_interfaces/msg/Time";

  explicit Time_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sec = 0l;
      this->nanosec = 0ul;
    }
  }

  explicit Time_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sec = 0l;
      this->nanosec = 0ul;
    }
  }

  // field types and members
  using _sec_type =
    int32_t;
  _sec_type sec;
  using _nanosec_type =
    uint32_t;
  _nanosec_type nanosec;

  // setters for named parameter idiom
  Type & set__sec(
    const int32_t & _arg)
  {
    this->sec = _arg;
    return *this;
  }
  Type & set__nanosec(
    const uint32_t & _arg)
  {
    this->nanosec = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    builtin_interfaces::msg::Time_<ContainerAllocator> *;
  using ConstRawPtr =
    const builtin_interfaces::msg::Time_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<builtin_interfaces::msg::Time_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<builtin_interfaces::msg::Time_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      builtin_interfaces::msg::Time_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<builtin_interfaces::msg::Time_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      builtin_interfaces::msg::Time_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<builtin_interfaces::msg::Time_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<builtin_interfaces::msg::Time_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<builtin_interfaces::msg::Time_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__builtin_interfaces__msg__Time
    std::shared_ptr<builtin_interfaces::msg::Time_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__builtin_interfaces__msg__Time
    std::shared_ptr<builtin_interfaces::msg::Time_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Time_ & other) const
  {
    if (this->sec != other.sec) {
      return false;
    }
    if (this->nanosec != other.nanosec) {
      return false;
    }
    return true;
  }
  bool operator!=(const Time_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Time_

// alias to use template instance with default allocator
using Time =
  builtin_interfaces::msg::Time_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace builtin_interfaces

#endif  // DIMOS_CDR_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Header.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/header.hpp"


#ifndef DIMOS_CDR_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D
#define DIMOS_CDR_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'stamp'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Header __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Header __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Header_
{
  using Type = Header_<ContainerAllocator>;
void validate() const {
stamp.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Header";

  explicit Header_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
    }
  }

  explicit Header_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    frame_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_id_type frame_id;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Header_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Header_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Header_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Header_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Header_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Header_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Header_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Header_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Header_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Header_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Header
    std::shared_ptr<std_msgs::msg::Header_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Header
    std::shared_ptr<std_msgs::msg::Header_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Header_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->frame_id != other.frame_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Header_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Header_

// alias to use template instance with default allocator
using Header =
  std_msgs::msg::Header_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/Point2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/point2_d.hpp"


#ifndef DIMOS_CDR_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51
#define DIMOS_CDR_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__Point2D __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__Point2D __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Point2D_
{
  using Type = Point2D_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "vision_msgs/msg/Point2D";

  explicit Point2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit Point2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::Point2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::Point2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::Point2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::Point2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Point2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Point2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Point2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Point2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::Point2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::Point2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__Point2D
    std::shared_ptr<vision_msgs::msg::Point2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__Point2D
    std::shared_ptr<vision_msgs::msg::Point2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Point2D_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Point2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Point2D_

// alias to use template instance with default allocator
using Point2D =
  vision_msgs::msg::Point2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/Pose2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/pose2_d.hpp"


#ifndef DIMOS_CDR_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97
#define DIMOS_CDR_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'position'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__Pose2D __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__Pose2D __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pose2D_
{
  using Type = Pose2D_<ContainerAllocator>;
void validate() const {
position.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/Pose2D";

  explicit Pose2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->theta = 0.0;
    }
  }

  explicit Pose2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->theta = 0.0;
    }
  }

  // field types and members
  using _position_type =
    vision_msgs::msg::Point2D_<ContainerAllocator>;
  _position_type position;
  using _theta_type =
    double;
  _theta_type theta;

  // setters for named parameter idiom
  Type & set__position(
    const vision_msgs::msg::Point2D_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::Pose2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::Pose2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::Pose2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::Pose2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Pose2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Pose2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Pose2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Pose2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::Pose2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::Pose2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__Pose2D
    std::shared_ptr<vision_msgs::msg::Pose2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__Pose2D
    std::shared_ptr<vision_msgs::msg::Pose2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pose2D_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pose2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pose2D_

// alias to use template instance with default allocator
using Pose2D =
  vision_msgs::msg::Pose2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/BoundingBox2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/bounding_box2_d.hpp"


#ifndef DIMOS_CDR_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA
#define DIMOS_CDR_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'center'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__BoundingBox2D __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__BoundingBox2D __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox2D_
{
  using Type = BoundingBox2D_<ContainerAllocator>;
void validate() const {
center.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/BoundingBox2D";

  explicit BoundingBox2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : center(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->size_x = 0.0;
      this->size_y = 0.0;
    }
  }

  explicit BoundingBox2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : center(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->size_x = 0.0;
      this->size_y = 0.0;
    }
  }

  // field types and members
  using _center_type =
    vision_msgs::msg::Pose2D_<ContainerAllocator>;
  _center_type center;
  using _size_x_type =
    double;
  _size_x_type size_x;
  using _size_y_type =
    double;
  _size_y_type size_y;

  // setters for named parameter idiom
  Type & set__center(
    const vision_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->center = _arg;
    return *this;
  }
  Type & set__size_x(
    const double & _arg)
  {
    this->size_x = _arg;
    return *this;
  }
  Type & set__size_y(
    const double & _arg)
  {
    this->size_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::BoundingBox2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::BoundingBox2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::BoundingBox2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::BoundingBox2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__BoundingBox2D
    std::shared_ptr<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__BoundingBox2D
    std::shared_ptr<vision_msgs::msg::BoundingBox2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox2D_ & other) const
  {
    if (this->center != other.center) {
      return false;
    }
    if (this->size_x != other.size_x) {
      return false;
    }
    if (this->size_y != other.size_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox2D_

// alias to use template instance with default allocator
using BoundingBox2D =
  vision_msgs::msg::BoundingBox2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/BoundingBox2DArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/bounding_box2_d_array.hpp"


#ifndef DIMOS_CDR_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A
#define DIMOS_CDR_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'boxes'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__BoundingBox2DArray __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__BoundingBox2DArray __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox2DArray_
{
  using Type = BoundingBox2DArray_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : boxes) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/BoundingBox2DArray";

  explicit BoundingBox2DArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit BoundingBox2DArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _boxes_type =
    std::vector<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>>;
  _boxes_type boxes;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__boxes(
    const std::vector<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>> & _arg)
  {
    this->boxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__BoundingBox2DArray
    std::shared_ptr<dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__BoundingBox2DArray
    std::shared_ptr<dimos_msgs::msg::BoundingBox2DArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox2DArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->boxes != other.boxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox2DArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox2DArray_

// alias to use template instance with default allocator
using BoundingBox2DArray =
  dimos_msgs::msg::BoundingBox2DArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Point.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/point.hpp"


#ifndef DIMOS_CDR_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C
#define DIMOS_CDR_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Point __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Point __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Point_
{
  using Type = Point_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "geometry_msgs/msg/Point";

  explicit Point_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  explicit Point_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Point_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Point_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Point_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Point_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Point_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Point_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Point_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Point_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Point_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Point_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Point
    std::shared_ptr<geometry_msgs::msg::Point_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Point
    std::shared_ptr<geometry_msgs::msg::Point_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Point_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const Point_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Point_

// alias to use template instance with default allocator
using Point =
  geometry_msgs::msg::Point_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Quaternion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/quaternion.hpp"


#ifndef DIMOS_CDR_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD
#define DIMOS_CDR_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Quaternion __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Quaternion __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Quaternion_
{
  using Type = Quaternion_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "geometry_msgs/msg/Quaternion";

  explicit Quaternion_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->w = 1.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->w = 0.0;
    }
  }

  explicit Quaternion_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->w = 1.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->w = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _w_type =
    double;
  _w_type w;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__w(
    const double & _arg)
  {
    this->w = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Quaternion_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Quaternion_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Quaternion_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Quaternion_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Quaternion_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Quaternion_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Quaternion_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Quaternion_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Quaternion_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Quaternion
    std::shared_ptr<geometry_msgs::msg::Quaternion_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Quaternion
    std::shared_ptr<geometry_msgs::msg::Quaternion_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Quaternion_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    return true;
  }
  bool operator!=(const Quaternion_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Quaternion_

// alias to use template instance with default allocator
using Quaternion =
  geometry_msgs::msg::Quaternion_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/pose.hpp"


#ifndef DIMOS_CDR_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827
#define DIMOS_CDR_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'position'
// Member 'orientation'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Pose __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Pose __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pose_
{
  using Type = Pose_<ContainerAllocator>;
void validate() const {
position.validate();
orientation.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Pose";

  explicit Pose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init),
    orientation(_init)
  {
    (void)_init;
  }

  explicit Pose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init),
    orientation(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;

  // setters for named parameter idiom
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Pose_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Pose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Pose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Pose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Pose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Pose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Pose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Pose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Pose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Pose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Pose
    std::shared_ptr<geometry_msgs::msg::Pose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Pose
    std::shared_ptr<geometry_msgs::msg::Pose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pose_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pose_

// alias to use template instance with default allocator
using Pose =
  geometry_msgs::msg::Pose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Vector3.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/vector3.hpp"


#ifndef DIMOS_CDR_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02
#define DIMOS_CDR_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Vector3 __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Vector3 __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Vector3_
{
  using Type = Vector3_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "geometry_msgs/msg/Vector3";

  explicit Vector3_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  explicit Vector3_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Vector3_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Vector3_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Vector3_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Vector3_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Vector3_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Vector3_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Vector3_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Vector3_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Vector3_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Vector3_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Vector3
    std::shared_ptr<geometry_msgs::msg::Vector3_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Vector3
    std::shared_ptr<geometry_msgs::msg::Vector3_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Vector3_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const Vector3_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Vector3_

// alias to use template instance with default allocator
using Vector3 =
  geometry_msgs::msg::Vector3_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/BoundingBox3D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/bounding_box3_d.hpp"


#ifndef DIMOS_CDR_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0
#define DIMOS_CDR_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'center'
// Member 'size'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__BoundingBox3D __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__BoundingBox3D __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox3D_
{
  using Type = BoundingBox3D_<ContainerAllocator>;
void validate() const {
center.validate();
size.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/BoundingBox3D";

  explicit BoundingBox3D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : center(_init),
    size(_init)
  {
    (void)_init;
  }

  explicit BoundingBox3D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : center(_alloc, _init),
    size(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _center_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _center_type center;
  using _size_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _size_type size;

  // setters for named parameter idiom
  Type & set__center(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->center = _arg;
    return *this;
  }
  Type & set__size(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->size = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::BoundingBox3D_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::BoundingBox3D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox3D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::BoundingBox3D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::BoundingBox3D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__BoundingBox3D
    std::shared_ptr<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__BoundingBox3D
    std::shared_ptr<vision_msgs::msg::BoundingBox3D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox3D_ & other) const
  {
    if (this->center != other.center) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox3D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox3D_

// alias to use template instance with default allocator
using BoundingBox3D =
  vision_msgs::msg::BoundingBox3D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/BoundingBox3DArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/bounding_box3_d_array.hpp"


#ifndef DIMOS_CDR_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A
#define DIMOS_CDR_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'boxes'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__BoundingBox3DArray __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__BoundingBox3DArray __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox3DArray_
{
  using Type = BoundingBox3DArray_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : boxes) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/BoundingBox3DArray";

  explicit BoundingBox3DArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit BoundingBox3DArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _boxes_type =
    std::vector<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>>;
  _boxes_type boxes;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__boxes(
    const std::vector<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>> & _arg)
  {
    this->boxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__BoundingBox3DArray
    std::shared_ptr<dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__BoundingBox3DArray
    std::shared_ptr<dimos_msgs::msg::BoundingBox3DArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox3DArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->boxes != other.boxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox3DArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox3DArray_

// alias to use template instance with default allocator
using BoundingBox3DArray =
  dimos_msgs::msg::BoundingBox3DArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/EntityMarker.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/entity_marker.hpp"


#ifndef DIMOS_CDR_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87
#define DIMOS_CDR_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'position'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__EntityMarker __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__EntityMarker __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EntityMarker_
{
  using Type = EntityMarker_<ContainerAllocator>;
void validate() const {
position.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/EntityMarker";

  explicit EntityMarker_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->entity_id = "";
      this->label = "";
      this->entity_type = "";
    }
  }

  explicit EntityMarker_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : entity_id(_alloc),
    label(_alloc),
    entity_type(_alloc),
    position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->entity_id = "";
      this->label = "";
      this->entity_type = "";
    }
  }

  // field types and members
  using _entity_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _entity_id_type entity_id;
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _label_type label;
  using _entity_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _entity_type_type entity_type;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__entity_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->entity_id = _arg;
    return *this;
  }
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__entity_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->entity_type = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::EntityMarker_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::EntityMarker_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::EntityMarker_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::EntityMarker_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::EntityMarker_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::EntityMarker_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::EntityMarker_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::EntityMarker_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::EntityMarker_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::EntityMarker_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__EntityMarker
    std::shared_ptr<dimos_msgs::msg::EntityMarker_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__EntityMarker
    std::shared_ptr<dimos_msgs::msg::EntityMarker_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EntityMarker_ & other) const
  {
    if (this->entity_id != other.entity_id) {
      return false;
    }
    if (this->label != other.label) {
      return false;
    }
    if (this->entity_type != other.entity_type) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const EntityMarker_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EntityMarker_

// alias to use template instance with default allocator
using EntityMarker =
  dimos_msgs::msg::EntityMarker_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/EntityMarkers.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/entity_markers.hpp"


#ifndef DIMOS_CDR_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254
#define DIMOS_CDR_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'markers'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__EntityMarkers __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__EntityMarkers __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EntityMarkers_
{
  using Type = EntityMarkers_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : markers) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/EntityMarkers";

  explicit EntityMarkers_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit EntityMarkers_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _markers_type =
    std::vector<dimos_msgs::msg::EntityMarker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dimos_msgs::msg::EntityMarker_<ContainerAllocator>>>;
  _markers_type markers;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__markers(
    const std::vector<dimos_msgs::msg::EntityMarker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dimos_msgs::msg::EntityMarker_<ContainerAllocator>>> & _arg)
  {
    this->markers = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::EntityMarkers_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::EntityMarkers_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::EntityMarkers_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::EntityMarkers_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::EntityMarkers_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::EntityMarkers_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::EntityMarkers_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::EntityMarkers_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::EntityMarkers_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::EntityMarkers_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__EntityMarkers
    std::shared_ptr<dimos_msgs::msg::EntityMarkers_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__EntityMarkers
    std::shared_ptr<dimos_msgs::msg::EntityMarkers_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EntityMarkers_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->markers != other.markers) {
      return false;
    }
    return true;
  }
  bool operator!=(const EntityMarkers_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EntityMarkers_

// alias to use template instance with default allocator
using EntityMarkers =
  dimos_msgs::msg::EntityMarkers_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/GraspCandidate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/grasp_candidate.hpp"


#ifndef DIMOS_CDR_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6
#define DIMOS_CDR_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'pose'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__GraspCandidate __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__GraspCandidate __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GraspCandidate_
{
  using Type = GraspCandidate_<ContainerAllocator>;
void validate() const {
pose.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/GraspCandidate";

  explicit GraspCandidate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->score = 0.0;
    }
  }

  explicit GraspCandidate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->score = 0.0;
    }
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _score_type =
    double;
  _score_type score;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__score(
    const double & _arg)
  {
    this->score = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::GraspCandidate_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::GraspCandidate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::GraspCandidate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::GraspCandidate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::GraspCandidate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::GraspCandidate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::GraspCandidate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::GraspCandidate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::GraspCandidate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::GraspCandidate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__GraspCandidate
    std::shared_ptr<dimos_msgs::msg::GraspCandidate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__GraspCandidate
    std::shared_ptr<dimos_msgs::msg::GraspCandidate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraspCandidate_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraspCandidate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraspCandidate_

// alias to use template instance with default allocator
using GraspCandidate =
  dimos_msgs::msg::GraspCandidate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/GraspCandidateArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/grasp_candidate_array.hpp"


#ifndef DIMOS_CDR_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC
#define DIMOS_CDR_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'candidates'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__GraspCandidateArray __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__GraspCandidateArray __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GraspCandidateArray_
{
  using Type = GraspCandidateArray_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : candidates) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/GraspCandidateArray";

  explicit GraspCandidateArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit GraspCandidateArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _candidates_type =
    std::vector<dimos_msgs::msg::GraspCandidate_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dimos_msgs::msg::GraspCandidate_<ContainerAllocator>>>;
  _candidates_type candidates;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__candidates(
    const std::vector<dimos_msgs::msg::GraspCandidate_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dimos_msgs::msg::GraspCandidate_<ContainerAllocator>>> & _arg)
  {
    this->candidates = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__GraspCandidateArray
    std::shared_ptr<dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__GraspCandidateArray
    std::shared_ptr<dimos_msgs::msg::GraspCandidateArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraspCandidateArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->candidates != other.candidates) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraspCandidateArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraspCandidateArray_

// alias to use template instance with default allocator
using GraspCandidateArray =
  dimos_msgs::msg::GraspCandidateArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/ImuInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/imu_info.hpp"


#ifndef DIMOS_CDR_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533
#define DIMOS_CDR_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__ImuInfo __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__ImuInfo __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuInfo_
{
  using Type = ImuInfo_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/ImuInfo";

  explicit ImuInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gyro_noise_density = 0.0;
      this->gyro_random_walk = 0.0;
      this->accel_noise_density = 0.0;
      this->accel_random_walk = 0.0;
      this->frequency = 0.0;
    }
  }

  explicit ImuInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gyro_noise_density = 0.0;
      this->gyro_random_walk = 0.0;
      this->accel_noise_density = 0.0;
      this->accel_random_walk = 0.0;
      this->frequency = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _gyro_noise_density_type =
    double;
  _gyro_noise_density_type gyro_noise_density;
  using _gyro_random_walk_type =
    double;
  _gyro_random_walk_type gyro_random_walk;
  using _accel_noise_density_type =
    double;
  _accel_noise_density_type accel_noise_density;
  using _accel_random_walk_type =
    double;
  _accel_random_walk_type accel_random_walk;
  using _frequency_type =
    double;
  _frequency_type frequency;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__gyro_noise_density(
    const double & _arg)
  {
    this->gyro_noise_density = _arg;
    return *this;
  }
  Type & set__gyro_random_walk(
    const double & _arg)
  {
    this->gyro_random_walk = _arg;
    return *this;
  }
  Type & set__accel_noise_density(
    const double & _arg)
  {
    this->accel_noise_density = _arg;
    return *this;
  }
  Type & set__accel_random_walk(
    const double & _arg)
  {
    this->accel_random_walk = _arg;
    return *this;
  }
  Type & set__frequency(
    const double & _arg)
  {
    this->frequency = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::ImuInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::ImuInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::ImuInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::ImuInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::ImuInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::ImuInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::ImuInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::ImuInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::ImuInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::ImuInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__ImuInfo
    std::shared_ptr<dimos_msgs::msg::ImuInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__ImuInfo
    std::shared_ptr<dimos_msgs::msg::ImuInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->gyro_noise_density != other.gyro_noise_density) {
      return false;
    }
    if (this->gyro_random_walk != other.gyro_random_walk) {
      return false;
    }
    if (this->accel_noise_density != other.accel_noise_density) {
      return false;
    }
    if (this->accel_random_walk != other.accel_random_walk) {
      return false;
    }
    if (this->frequency != other.frequency) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuInfo_

// alias to use template instance with default allocator
using ImuInfo =
  dimos_msgs::msg::ImuInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/JointCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/joint_command.hpp"


#ifndef DIMOS_CDR_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02
#define DIMOS_CDR_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__JointCommand __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__JointCommand __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointCommand_
{
  using Type = JointCommand_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/JointCommand";

  explicit JointCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit JointCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _positions_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _positions_type positions;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__positions(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->positions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::JointCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::JointCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::JointCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::JointCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::JointCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::JointCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::JointCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::JointCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::JointCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::JointCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__JointCommand
    std::shared_ptr<dimos_msgs::msg::JointCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__JointCommand
    std::shared_ptr<dimos_msgs::msg::JointCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->positions != other.positions) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointCommand_

// alias to use template instance with default allocator
using JointCommand =
  dimos_msgs::msg::JointCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/LineSegment3D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/line_segment3_d.hpp"


#ifndef DIMOS_CDR_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2
#define DIMOS_CDR_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'start'
// Member 'end'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__LineSegment3D __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__LineSegment3D __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LineSegment3D_
{
  using Type = LineSegment3D_<ContainerAllocator>;
void validate() const {
start.validate();
end.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/LineSegment3D";

  explicit LineSegment3D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_init),
    end(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->weight = 1.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->weight = 0.0;
    }
  }

  explicit LineSegment3D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start(_alloc, _init),
    end(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->weight = 1.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->weight = 0.0;
    }
  }

  // field types and members
  using _start_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _start_type start;
  using _end_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _end_type end;
  using _weight_type =
    double;
  _weight_type weight;

  // setters for named parameter idiom
  Type & set__start(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__end(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->end = _arg;
    return *this;
  }
  Type & set__weight(
    const double & _arg)
  {
    this->weight = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::LineSegment3D_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::LineSegment3D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::LineSegment3D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::LineSegment3D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::LineSegment3D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::LineSegment3D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::LineSegment3D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::LineSegment3D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::LineSegment3D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::LineSegment3D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__LineSegment3D
    std::shared_ptr<dimos_msgs::msg::LineSegment3D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__LineSegment3D
    std::shared_ptr<dimos_msgs::msg::LineSegment3D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LineSegment3D_ & other) const
  {
    if (this->start != other.start) {
      return false;
    }
    if (this->end != other.end) {
      return false;
    }
    if (this->weight != other.weight) {
      return false;
    }
    return true;
  }
  bool operator!=(const LineSegment3D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LineSegment3D_

// alias to use template instance with default allocator
using LineSegment3D =
  dimos_msgs::msg::LineSegment3D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/LineSegments3D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/line_segments3_d.hpp"


#ifndef DIMOS_CDR_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C
#define DIMOS_CDR_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'segments'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__LineSegments3D __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__LineSegments3D __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LineSegments3D_
{
  using Type = LineSegments3D_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : segments) { item.validate(); }
}
static constexpr const char* msg_name = "dimos_msgs/msg/LineSegments3D";

  explicit LineSegments3D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit LineSegments3D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _segments_type =
    std::vector<dimos_msgs::msg::LineSegment3D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dimos_msgs::msg::LineSegment3D_<ContainerAllocator>>>;
  _segments_type segments;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__segments(
    const std::vector<dimos_msgs::msg::LineSegment3D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dimos_msgs::msg::LineSegment3D_<ContainerAllocator>>> & _arg)
  {
    this->segments = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::LineSegments3D_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::LineSegments3D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::LineSegments3D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::LineSegments3D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::LineSegments3D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::LineSegments3D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::LineSegments3D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::LineSegments3D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::LineSegments3D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::LineSegments3D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__LineSegments3D
    std::shared_ptr<dimos_msgs::msg::LineSegments3D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__LineSegments3D
    std::shared_ptr<dimos_msgs::msg::LineSegments3D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LineSegments3D_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->segments != other.segments) {
      return false;
    }
    return true;
  }
  bool operator!=(const LineSegments3D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LineSegments3D_

// alias to use template instance with default allocator
using LineSegments3D =
  dimos_msgs::msg::LineSegments3D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/MotorCommandArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/motor_command_array.hpp"


#ifndef DIMOS_CDR_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03
#define DIMOS_CDR_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__MotorCommandArray __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__MotorCommandArray __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCommandArray_
{
  using Type = MotorCommandArray_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/MotorCommandArray";

  explicit MotorCommandArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MotorCommandArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _q_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _q_type q;
  using _dq_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _dq_type dq;
  using _kp_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _kp_type kp;
  using _kd_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _kd_type kd;
  using _tau_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _tau_type tau;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__q(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->q = _arg;
    return *this;
  }
  Type & set__dq(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->dq = _arg;
    return *this;
  }
  Type & set__kp(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__tau(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->tau = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::MotorCommandArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::MotorCommandArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::MotorCommandArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::MotorCommandArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::MotorCommandArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::MotorCommandArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::MotorCommandArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::MotorCommandArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::MotorCommandArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::MotorCommandArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__MotorCommandArray
    std::shared_ptr<dimos_msgs::msg::MotorCommandArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__MotorCommandArray
    std::shared_ptr<dimos_msgs::msg::MotorCommandArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommandArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->q != other.q) {
      return false;
    }
    if (this->dq != other.dq) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->tau != other.tau) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommandArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommandArray_

// alias to use template instance with default allocator
using MotorCommandArray =
  dimos_msgs::msg::MotorCommandArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/robot_state.hpp"


#ifndef DIMOS_CDR_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3
#define DIMOS_CDR_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__RobotState __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__RobotState __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotState_
{
  using Type = RobotState_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/RobotState";

  explicit RobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0l;
      this->mode = 0l;
      this->error_code = 0l;
      this->warn_code = 0l;
      this->cmdnum = 0l;
      this->mt_brake = 0l;
      this->mt_able = 0l;
    }
  }

  explicit RobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0l;
      this->mode = 0l;
      this->error_code = 0l;
      this->warn_code = 0l;
      this->cmdnum = 0l;
      this->mt_brake = 0l;
      this->mt_able = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    int32_t;
  _state_type state;
  using _mode_type =
    int32_t;
  _mode_type mode;
  using _error_code_type =
    int32_t;
  _error_code_type error_code;
  using _warn_code_type =
    int32_t;
  _warn_code_type warn_code;
  using _cmdnum_type =
    int32_t;
  _cmdnum_type cmdnum;
  using _mt_brake_type =
    int32_t;
  _mt_brake_type mt_brake;
  using _mt_able_type =
    int32_t;
  _mt_able_type mt_able;
  using _tcp_pose_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _tcp_pose_type tcp_pose;
  using _tcp_offset_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _tcp_offset_type tcp_offset;
  using _joints_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joints_type joints;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const int32_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__mode(
    const int32_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__error_code(
    const int32_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__warn_code(
    const int32_t & _arg)
  {
    this->warn_code = _arg;
    return *this;
  }
  Type & set__cmdnum(
    const int32_t & _arg)
  {
    this->cmdnum = _arg;
    return *this;
  }
  Type & set__mt_brake(
    const int32_t & _arg)
  {
    this->mt_brake = _arg;
    return *this;
  }
  Type & set__mt_able(
    const int32_t & _arg)
  {
    this->mt_able = _arg;
    return *this;
  }
  Type & set__tcp_pose(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->tcp_pose = _arg;
    return *this;
  }
  Type & set__tcp_offset(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->tcp_offset = _arg;
    return *this;
  }
  Type & set__joints(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::RobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::RobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::RobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::RobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::RobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::RobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::RobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::RobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__RobotState
    std::shared_ptr<dimos_msgs::msg::RobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__RobotState
    std::shared_ptr<dimos_msgs::msg::RobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->warn_code != other.warn_code) {
      return false;
    }
    if (this->cmdnum != other.cmdnum) {
      return false;
    }
    if (this->mt_brake != other.mt_brake) {
      return false;
    }
    if (this->mt_able != other.mt_able) {
      return false;
    }
    if (this->tcp_pose != other.tcp_pose) {
      return false;
    }
    if (this->tcp_offset != other.tcp_offset) {
      return false;
    }
    if (this->joints != other.joints) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotState_

// alias to use template instance with default allocator
using RobotState =
  dimos_msgs::msg::RobotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/TrajectoryStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/trajectory_status.hpp"


#ifndef DIMOS_CDR_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64
#define DIMOS_CDR_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'time_elapsed'
// Member 'time_remaining'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__TrajectoryStatus __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__TrajectoryStatus __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryStatus_
{
  using Type = TrajectoryStatus_<ContainerAllocator>;
void validate() const {
header.validate();
time_elapsed.validate();
time_remaining.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/TrajectoryStatus";

  explicit TrajectoryStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    time_elapsed(_init),
    time_remaining(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->progress = 0.0;
      this->error = "";
    }
  }

  explicit TrajectoryStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    time_elapsed(_alloc, _init),
    time_remaining(_alloc, _init),
    error(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->progress = 0.0;
      this->error = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;
  using _progress_type =
    double;
  _progress_type progress;
  using _time_elapsed_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _time_elapsed_type time_elapsed;
  using _time_remaining_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _time_remaining_type time_remaining;
  using _error_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_type error;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__progress(
    const double & _arg)
  {
    this->progress = _arg;
    return *this;
  }
  Type & set__time_elapsed(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->time_elapsed = _arg;
    return *this;
  }
  Type & set__time_remaining(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->time_remaining = _arg;
    return *this;
  }
  Type & set__error(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t IDLE =
    0u;
  static constexpr uint8_t EXECUTING =
    1u;
  static constexpr uint8_t COMPLETED =
    2u;
  static constexpr uint8_t ABORTED =
    3u;
  static constexpr uint8_t FAULT =
    4u;

  // pointer types
  using RawPtr =
    dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__TrajectoryStatus
    std::shared_ptr<dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__TrajectoryStatus
    std::shared_ptr<dimos_msgs::msg::TrajectoryStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->progress != other.progress) {
      return false;
    }
    if (this->time_elapsed != other.time_elapsed) {
      return false;
    }
    if (this->time_remaining != other.time_remaining) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryStatus_

// alias to use template instance with default allocator
using TrajectoryStatus =
  dimos_msgs::msg::TrajectoryStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TrajectoryStatus_<ContainerAllocator>::IDLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TrajectoryStatus_<ContainerAllocator>::EXECUTING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TrajectoryStatus_<ContainerAllocator>::COMPLETED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TrajectoryStatus_<ContainerAllocator>::ABORTED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TrajectoryStatus_<ContainerAllocator>::FAULT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dimos_msgs:msg/VideoStats.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dimos_msgs/msg/video_stats.hpp"


#ifndef DIMOS_CDR_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125
#define DIMOS_CDR_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__dimos_msgs__msg__VideoStats __attribute__((deprecated))
#else
# define DEPRECATED__dimos_msgs__msg__VideoStats __declspec(deprecated)
#endif

namespace dimos_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VideoStats_
{
  using Type = VideoStats_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "dimos_msgs/msg/VideoStats";

  explicit VideoStats_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fps = 0.0;
      this->kbps = 0.0;
      this->width = 0ul;
      this->height = 0ul;
      this->loss_pct = 0.0;
      this->jitter_buffer_ms = 0.0;
      this->decode_ms = 0.0;
      this->frames_dropped = 0ull;
      this->freezes = 0ull;
      this->e2e_latency_ms = 0.0;
    }
  }

  explicit VideoStats_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fps = 0.0;
      this->kbps = 0.0;
      this->width = 0ul;
      this->height = 0ul;
      this->loss_pct = 0.0;
      this->jitter_buffer_ms = 0.0;
      this->decode_ms = 0.0;
      this->frames_dropped = 0ull;
      this->freezes = 0ull;
      this->e2e_latency_ms = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fps_type =
    double;
  _fps_type fps;
  using _kbps_type =
    double;
  _kbps_type kbps;
  using _width_type =
    uint32_t;
  _width_type width;
  using _height_type =
    uint32_t;
  _height_type height;
  using _loss_pct_type =
    double;
  _loss_pct_type loss_pct;
  using _jitter_buffer_ms_type =
    double;
  _jitter_buffer_ms_type jitter_buffer_ms;
  using _decode_ms_type =
    double;
  _decode_ms_type decode_ms;
  using _frames_dropped_type =
    uint64_t;
  _frames_dropped_type frames_dropped;
  using _freezes_type =
    uint64_t;
  _freezes_type freezes;
  using _e2e_latency_ms_type =
    double;
  _e2e_latency_ms_type e2e_latency_ms;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__fps(
    const double & _arg)
  {
    this->fps = _arg;
    return *this;
  }
  Type & set__kbps(
    const double & _arg)
  {
    this->kbps = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__loss_pct(
    const double & _arg)
  {
    this->loss_pct = _arg;
    return *this;
  }
  Type & set__jitter_buffer_ms(
    const double & _arg)
  {
    this->jitter_buffer_ms = _arg;
    return *this;
  }
  Type & set__decode_ms(
    const double & _arg)
  {
    this->decode_ms = _arg;
    return *this;
  }
  Type & set__frames_dropped(
    const uint64_t & _arg)
  {
    this->frames_dropped = _arg;
    return *this;
  }
  Type & set__freezes(
    const uint64_t & _arg)
  {
    this->freezes = _arg;
    return *this;
  }
  Type & set__e2e_latency_ms(
    const double & _arg)
  {
    this->e2e_latency_ms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dimos_msgs::msg::VideoStats_<ContainerAllocator> *;
  using ConstRawPtr =
    const dimos_msgs::msg::VideoStats_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dimos_msgs::msg::VideoStats_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dimos_msgs::msg::VideoStats_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::VideoStats_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::VideoStats_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dimos_msgs::msg::VideoStats_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dimos_msgs::msg::VideoStats_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dimos_msgs::msg::VideoStats_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dimos_msgs::msg::VideoStats_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dimos_msgs__msg__VideoStats
    std::shared_ptr<dimos_msgs::msg::VideoStats_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dimos_msgs__msg__VideoStats
    std::shared_ptr<dimos_msgs::msg::VideoStats_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VideoStats_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fps != other.fps) {
      return false;
    }
    if (this->kbps != other.kbps) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->loss_pct != other.loss_pct) {
      return false;
    }
    if (this->jitter_buffer_ms != other.jitter_buffer_ms) {
      return false;
    }
    if (this->decode_ms != other.decode_ms) {
      return false;
    }
    if (this->frames_dropped != other.frames_dropped) {
      return false;
    }
    if (this->freezes != other.freezes) {
      return false;
    }
    if (this->e2e_latency_ms != other.e2e_latency_ms) {
      return false;
    }
    return true;
  }
  bool operator!=(const VideoStats_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VideoStats_

// alias to use template instance with default allocator
using VideoStats =
  dimos_msgs::msg::VideoStats_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dimos_msgs

#endif  // DIMOS_CDR_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from foxglove_msgs:msg/CompressedVideo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "foxglove_msgs/msg/compressed_video.hpp"


#ifndef DIMOS_CDR_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82
#define DIMOS_CDR_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'timestamp'

#ifndef _WIN32
# define DEPRECATED__foxglove_msgs__msg__CompressedVideo __attribute__((deprecated))
#else
# define DEPRECATED__foxglove_msgs__msg__CompressedVideo __declspec(deprecated)
#endif

namespace foxglove_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CompressedVideo_
{
  using Type = CompressedVideo_<ContainerAllocator>;
void validate() const {
timestamp.validate();
}
static constexpr const char* msg_name = "foxglove_msgs/msg/CompressedVideo";

  explicit CompressedVideo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->format = "";
    }
  }

  explicit CompressedVideo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    frame_id(_alloc),
    format(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->format = "";
    }
  }

  // field types and members
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_id_type frame_id;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;
  using _format_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _format_type format;

  // setters for named parameter idiom
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__format(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->format = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    foxglove_msgs::msg::CompressedVideo_<ContainerAllocator> *;
  using ConstRawPtr =
    const foxglove_msgs::msg::CompressedVideo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<foxglove_msgs::msg::CompressedVideo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<foxglove_msgs::msg::CompressedVideo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      foxglove_msgs::msg::CompressedVideo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<foxglove_msgs::msg::CompressedVideo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      foxglove_msgs::msg::CompressedVideo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<foxglove_msgs::msg::CompressedVideo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<foxglove_msgs::msg::CompressedVideo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<foxglove_msgs::msg::CompressedVideo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__foxglove_msgs__msg__CompressedVideo
    std::shared_ptr<foxglove_msgs::msg::CompressedVideo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__foxglove_msgs__msg__CompressedVideo
    std::shared_ptr<foxglove_msgs::msg::CompressedVideo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CompressedVideo_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->frame_id != other.frame_id) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->format != other.format) {
      return false;
    }
    return true;
  }
  bool operator!=(const CompressedVideo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CompressedVideo_

// alias to use template instance with default allocator
using CompressedVideo =
  foxglove_msgs::msg::CompressedVideo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace foxglove_msgs

#endif  // DIMOS_CDR_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Accel.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/accel.hpp"


#ifndef DIMOS_CDR_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962
#define DIMOS_CDR_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'linear'
// Member 'angular'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Accel __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Accel __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Accel_
{
  using Type = Accel_<ContainerAllocator>;
void validate() const {
linear.validate();
angular.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Accel";

  explicit Accel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : linear(_init),
    angular(_init)
  {
    (void)_init;
  }

  explicit Accel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : linear(_alloc, _init),
    angular(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _linear_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_type linear;
  using _angular_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_type angular;

  // setters for named parameter idiom
  Type & set__linear(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear = _arg;
    return *this;
  }
  Type & set__angular(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Accel_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Accel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Accel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Accel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Accel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Accel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Accel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Accel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Accel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Accel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Accel
    std::shared_ptr<geometry_msgs::msg::Accel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Accel
    std::shared_ptr<geometry_msgs::msg::Accel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Accel_ & other) const
  {
    if (this->linear != other.linear) {
      return false;
    }
    if (this->angular != other.angular) {
      return false;
    }
    return true;
  }
  bool operator!=(const Accel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Accel_

// alias to use template instance with default allocator
using Accel =
  geometry_msgs::msg::Accel_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/AccelStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/accel_stamped.hpp"


#ifndef DIMOS_CDR_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3
#define DIMOS_CDR_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'accel'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__AccelStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__AccelStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AccelStamped_
{
  using Type = AccelStamped_<ContainerAllocator>;
void validate() const {
header.validate();
accel.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/AccelStamped";

  explicit AccelStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    accel(_init)
  {
    (void)_init;
  }

  explicit AccelStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    accel(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _accel_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _accel_type accel;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__accel(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->accel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::AccelStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::AccelStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::AccelStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::AccelStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::AccelStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::AccelStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::AccelStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::AccelStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::AccelStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::AccelStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__AccelStamped
    std::shared_ptr<geometry_msgs::msg::AccelStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__AccelStamped
    std::shared_ptr<geometry_msgs::msg::AccelStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AccelStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->accel != other.accel) {
      return false;
    }
    return true;
  }
  bool operator!=(const AccelStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AccelStamped_

// alias to use template instance with default allocator
using AccelStamped =
  geometry_msgs::msg::AccelStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/AccelWithCovariance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/accel_with_covariance.hpp"


#ifndef DIMOS_CDR_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7
#define DIMOS_CDR_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'accel'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__AccelWithCovariance __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__AccelWithCovariance __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AccelWithCovariance_
{
  using Type = AccelWithCovariance_<ContainerAllocator>;
void validate() const {
accel.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/AccelWithCovariance";

  explicit AccelWithCovariance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : accel(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 36>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  explicit AccelWithCovariance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : accel(_alloc, _init),
    covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 36>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  // field types and members
  using _accel_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _accel_type accel;
  using _covariance_type =
    std::array<double, 36>;
  _covariance_type covariance;

  // setters for named parameter idiom
  Type & set__accel(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->accel = _arg;
    return *this;
  }
  Type & set__covariance(
    const std::array<double, 36> & _arg)
  {
    this->covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__AccelWithCovariance
    std::shared_ptr<geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__AccelWithCovariance
    std::shared_ptr<geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AccelWithCovariance_ & other) const
  {
    if (this->accel != other.accel) {
      return false;
    }
    if (this->covariance != other.covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const AccelWithCovariance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AccelWithCovariance_

// alias to use template instance with default allocator
using AccelWithCovariance =
  geometry_msgs::msg::AccelWithCovariance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/AccelWithCovarianceStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"


#ifndef DIMOS_CDR_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283
#define DIMOS_CDR_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'accel'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__AccelWithCovarianceStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__AccelWithCovarianceStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AccelWithCovarianceStamped_
{
  using Type = AccelWithCovarianceStamped_<ContainerAllocator>;
void validate() const {
header.validate();
accel.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/AccelWithCovarianceStamped";

  explicit AccelWithCovarianceStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    accel(_init)
  {
    (void)_init;
  }

  explicit AccelWithCovarianceStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    accel(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _accel_type =
    geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator>;
  _accel_type accel;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__accel(
    const geometry_msgs::msg::AccelWithCovariance_<ContainerAllocator> & _arg)
  {
    this->accel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__AccelWithCovarianceStamped
    std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__AccelWithCovarianceStamped
    std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AccelWithCovarianceStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->accel != other.accel) {
      return false;
    }
    return true;
  }
  bool operator!=(const AccelWithCovarianceStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AccelWithCovarianceStamped_

// alias to use template instance with default allocator
using AccelWithCovarianceStamped =
  geometry_msgs::msg::AccelWithCovarianceStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Inertia.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/inertia.hpp"


#ifndef DIMOS_CDR_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE
#define DIMOS_CDR_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'com'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Inertia __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Inertia __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Inertia_
{
  using Type = Inertia_<ContainerAllocator>;
void validate() const {
com.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Inertia";

  explicit Inertia_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : com(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->m = 0.0;
      this->ixx = 0.0;
      this->ixy = 0.0;
      this->ixz = 0.0;
      this->iyy = 0.0;
      this->iyz = 0.0;
      this->izz = 0.0;
    }
  }

  explicit Inertia_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : com(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->m = 0.0;
      this->ixx = 0.0;
      this->ixy = 0.0;
      this->ixz = 0.0;
      this->iyy = 0.0;
      this->iyz = 0.0;
      this->izz = 0.0;
    }
  }

  // field types and members
  using _m_type =
    double;
  _m_type m;
  using _com_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _com_type com;
  using _ixx_type =
    double;
  _ixx_type ixx;
  using _ixy_type =
    double;
  _ixy_type ixy;
  using _ixz_type =
    double;
  _ixz_type ixz;
  using _iyy_type =
    double;
  _iyy_type iyy;
  using _iyz_type =
    double;
  _iyz_type iyz;
  using _izz_type =
    double;
  _izz_type izz;

  // setters for named parameter idiom
  Type & set__m(
    const double & _arg)
  {
    this->m = _arg;
    return *this;
  }
  Type & set__com(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->com = _arg;
    return *this;
  }
  Type & set__ixx(
    const double & _arg)
  {
    this->ixx = _arg;
    return *this;
  }
  Type & set__ixy(
    const double & _arg)
  {
    this->ixy = _arg;
    return *this;
  }
  Type & set__ixz(
    const double & _arg)
  {
    this->ixz = _arg;
    return *this;
  }
  Type & set__iyy(
    const double & _arg)
  {
    this->iyy = _arg;
    return *this;
  }
  Type & set__iyz(
    const double & _arg)
  {
    this->iyz = _arg;
    return *this;
  }
  Type & set__izz(
    const double & _arg)
  {
    this->izz = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Inertia_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Inertia_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Inertia_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Inertia_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Inertia_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Inertia_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Inertia_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Inertia_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Inertia_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Inertia_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Inertia
    std::shared_ptr<geometry_msgs::msg::Inertia_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Inertia
    std::shared_ptr<geometry_msgs::msg::Inertia_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Inertia_ & other) const
  {
    if (this->m != other.m) {
      return false;
    }
    if (this->com != other.com) {
      return false;
    }
    if (this->ixx != other.ixx) {
      return false;
    }
    if (this->ixy != other.ixy) {
      return false;
    }
    if (this->ixz != other.ixz) {
      return false;
    }
    if (this->iyy != other.iyy) {
      return false;
    }
    if (this->iyz != other.iyz) {
      return false;
    }
    if (this->izz != other.izz) {
      return false;
    }
    return true;
  }
  bool operator!=(const Inertia_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Inertia_

// alias to use template instance with default allocator
using Inertia =
  geometry_msgs::msg::Inertia_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/InertiaStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/inertia_stamped.hpp"


#ifndef DIMOS_CDR_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF
#define DIMOS_CDR_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'inertia'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__InertiaStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__InertiaStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InertiaStamped_
{
  using Type = InertiaStamped_<ContainerAllocator>;
void validate() const {
header.validate();
inertia.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/InertiaStamped";

  explicit InertiaStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    inertia(_init)
  {
    (void)_init;
  }

  explicit InertiaStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    inertia(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _inertia_type =
    geometry_msgs::msg::Inertia_<ContainerAllocator>;
  _inertia_type inertia;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__inertia(
    const geometry_msgs::msg::Inertia_<ContainerAllocator> & _arg)
  {
    this->inertia = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::InertiaStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::InertiaStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::InertiaStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::InertiaStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::InertiaStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::InertiaStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::InertiaStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::InertiaStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::InertiaStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::InertiaStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__InertiaStamped
    std::shared_ptr<geometry_msgs::msg::InertiaStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__InertiaStamped
    std::shared_ptr<geometry_msgs::msg::InertiaStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InertiaStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->inertia != other.inertia) {
      return false;
    }
    return true;
  }
  bool operator!=(const InertiaStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InertiaStamped_

// alias to use template instance with default allocator
using InertiaStamped =
  geometry_msgs::msg::InertiaStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Point32.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/point32.hpp"


#ifndef DIMOS_CDR_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6
#define DIMOS_CDR_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Point32 __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Point32 __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Point32_
{
  using Type = Point32_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "geometry_msgs/msg/Point32";

  explicit Point32_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }
  }

  explicit Point32_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Point32_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Point32_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Point32_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Point32_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Point32_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Point32_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Point32_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Point32_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Point32_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Point32_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Point32
    std::shared_ptr<geometry_msgs::msg::Point32_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Point32
    std::shared_ptr<geometry_msgs::msg::Point32_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Point32_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    return true;
  }
  bool operator!=(const Point32_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Point32_

// alias to use template instance with default allocator
using Point32 =
  geometry_msgs::msg::Point32_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/PointStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/point_stamped.hpp"


#ifndef DIMOS_CDR_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328
#define DIMOS_CDR_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'point'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__PointStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__PointStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointStamped_
{
  using Type = PointStamped_<ContainerAllocator>;
void validate() const {
header.validate();
point.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PointStamped";

  explicit PointStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    point(_init)
  {
    (void)_init;
  }

  explicit PointStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    point(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _point_type point;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::PointStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::PointStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::PointStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::PointStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PointStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PointStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PointStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PointStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::PointStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::PointStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__PointStamped
    std::shared_ptr<geometry_msgs::msg::PointStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__PointStamped
    std::shared_ptr<geometry_msgs::msg::PointStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->point != other.point) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointStamped_

// alias to use template instance with default allocator
using PointStamped =
  geometry_msgs::msg::PointStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Polygon.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/polygon.hpp"


#ifndef DIMOS_CDR_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469
#define DIMOS_CDR_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'points'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Polygon __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Polygon __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Polygon_
{
  using Type = Polygon_<ContainerAllocator>;
void validate() const {
for (const auto& item : points) { item.validate(); }
}
static constexpr const char* msg_name = "geometry_msgs/msg/Polygon";

  explicit Polygon_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Polygon_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _points_type =
    std::vector<geometry_msgs::msg::Point32_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point32_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point32_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point32_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Polygon_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Polygon_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Polygon_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Polygon_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Polygon_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Polygon_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Polygon_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Polygon_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Polygon_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Polygon_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Polygon
    std::shared_ptr<geometry_msgs::msg::Polygon_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Polygon
    std::shared_ptr<geometry_msgs::msg::Polygon_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Polygon_ & other) const
  {
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const Polygon_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Polygon_

// alias to use template instance with default allocator
using Polygon =
  geometry_msgs::msg::Polygon_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/PolygonInstance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/polygon_instance.hpp"


#ifndef DIMOS_CDR_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C
#define DIMOS_CDR_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'polygon'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__PolygonInstance __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__PolygonInstance __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PolygonInstance_
{
  using Type = PolygonInstance_<ContainerAllocator>;
void validate() const {
polygon.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PolygonInstance";

  explicit PolygonInstance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : polygon(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ll;
    }
  }

  explicit PolygonInstance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : polygon(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ll;
    }
  }

  // field types and members
  using _polygon_type =
    geometry_msgs::msg::Polygon_<ContainerAllocator>;
  _polygon_type polygon;
  using _id_type =
    int64_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__polygon(
    const geometry_msgs::msg::Polygon_<ContainerAllocator> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }
  Type & set__id(
    const int64_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::PolygonInstance_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::PolygonInstance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::PolygonInstance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::PolygonInstance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PolygonInstance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PolygonInstance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PolygonInstance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PolygonInstance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::PolygonInstance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::PolygonInstance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__PolygonInstance
    std::shared_ptr<geometry_msgs::msg::PolygonInstance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__PolygonInstance
    std::shared_ptr<geometry_msgs::msg::PolygonInstance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PolygonInstance_ & other) const
  {
    if (this->polygon != other.polygon) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PolygonInstance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PolygonInstance_

// alias to use template instance with default allocator
using PolygonInstance =
  geometry_msgs::msg::PolygonInstance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/PolygonInstanceStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/polygon_instance_stamped.hpp"


#ifndef DIMOS_CDR_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22
#define DIMOS_CDR_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'polygon'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__PolygonInstanceStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__PolygonInstanceStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PolygonInstanceStamped_
{
  using Type = PolygonInstanceStamped_<ContainerAllocator>;
void validate() const {
header.validate();
polygon.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PolygonInstanceStamped";

  explicit PolygonInstanceStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    polygon(_init)
  {
    (void)_init;
  }

  explicit PolygonInstanceStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    polygon(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _polygon_type =
    geometry_msgs::msg::PolygonInstance_<ContainerAllocator>;
  _polygon_type polygon;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__polygon(
    const geometry_msgs::msg::PolygonInstance_<ContainerAllocator> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__PolygonInstanceStamped
    std::shared_ptr<geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__PolygonInstanceStamped
    std::shared_ptr<geometry_msgs::msg::PolygonInstanceStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PolygonInstanceStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->polygon != other.polygon) {
      return false;
    }
    return true;
  }
  bool operator!=(const PolygonInstanceStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PolygonInstanceStamped_

// alias to use template instance with default allocator
using PolygonInstanceStamped =
  geometry_msgs::msg::PolygonInstanceStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/PolygonStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/polygon_stamped.hpp"


#ifndef DIMOS_CDR_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F
#define DIMOS_CDR_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'polygon'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__PolygonStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__PolygonStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PolygonStamped_
{
  using Type = PolygonStamped_<ContainerAllocator>;
void validate() const {
header.validate();
polygon.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PolygonStamped";

  explicit PolygonStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    polygon(_init)
  {
    (void)_init;
  }

  explicit PolygonStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    polygon(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _polygon_type =
    geometry_msgs::msg::Polygon_<ContainerAllocator>;
  _polygon_type polygon;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__polygon(
    const geometry_msgs::msg::Polygon_<ContainerAllocator> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::PolygonStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::PolygonStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::PolygonStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::PolygonStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PolygonStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PolygonStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PolygonStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PolygonStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::PolygonStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::PolygonStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__PolygonStamped
    std::shared_ptr<geometry_msgs::msg::PolygonStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__PolygonStamped
    std::shared_ptr<geometry_msgs::msg::PolygonStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PolygonStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->polygon != other.polygon) {
      return false;
    }
    return true;
  }
  bool operator!=(const PolygonStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PolygonStamped_

// alias to use template instance with default allocator
using PolygonStamped =
  geometry_msgs::msg::PolygonStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Pose2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/pose2_d.hpp"


#ifndef DIMOS_CDR_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8
#define DIMOS_CDR_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Pose2D __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Pose2D __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pose2D_
{
  using Type = Pose2D_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "geometry_msgs/msg/Pose2D";

  explicit Pose2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  explicit Pose2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->theta = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _theta_type =
    double;
  _theta_type theta;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Pose2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Pose2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Pose2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Pose2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Pose2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Pose2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Pose2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Pose2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Pose2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Pose2D
    std::shared_ptr<geometry_msgs::msg::Pose2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Pose2D
    std::shared_ptr<geometry_msgs::msg::Pose2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pose2D_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pose2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pose2D_

// alias to use template instance with default allocator
using Pose2D =
  geometry_msgs::msg::Pose2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/PoseArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/pose_array.hpp"


#ifndef DIMOS_CDR_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3
#define DIMOS_CDR_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'poses'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__PoseArray __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__PoseArray __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseArray_
{
  using Type = PoseArray_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : poses) { item.validate(); }
}
static constexpr const char* msg_name = "geometry_msgs/msg/PoseArray";

  explicit PoseArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit PoseArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _poses_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _poses_type poses;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__poses(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::PoseArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::PoseArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::PoseArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::PoseArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PoseArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PoseArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PoseArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PoseArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::PoseArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::PoseArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__PoseArray
    std::shared_ptr<geometry_msgs::msg::PoseArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__PoseArray
    std::shared_ptr<geometry_msgs::msg::PoseArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->poses != other.poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseArray_

// alias to use template instance with default allocator
using PoseArray =
  geometry_msgs::msg::PoseArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/PoseStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/pose_stamped.hpp"


#ifndef DIMOS_CDR_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F
#define DIMOS_CDR_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'pose'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__PoseStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__PoseStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseStamped_
{
  using Type = PoseStamped_<ContainerAllocator>;
void validate() const {
header.validate();
pose.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PoseStamped";

  explicit PoseStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    (void)_init;
  }

  explicit PoseStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::PoseStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PoseStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::PoseStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__PoseStamped
    std::shared_ptr<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__PoseStamped
    std::shared_ptr<geometry_msgs::msg::PoseStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseStamped_

// alias to use template instance with default allocator
using PoseStamped =
  geometry_msgs::msg::PoseStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/PoseWithCovariance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/pose_with_covariance.hpp"


#ifndef DIMOS_CDR_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A
#define DIMOS_CDR_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'pose'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__PoseWithCovariance __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__PoseWithCovariance __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseWithCovariance_
{
  using Type = PoseWithCovariance_<ContainerAllocator>;
void validate() const {
pose.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PoseWithCovariance";

  explicit PoseWithCovariance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 36>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  explicit PoseWithCovariance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 36>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _covariance_type =
    std::array<double, 36>;
  _covariance_type covariance;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__covariance(
    const std::array<double, 36> & _arg)
  {
    this->covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__PoseWithCovariance
    std::shared_ptr<geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__PoseWithCovariance
    std::shared_ptr<geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseWithCovariance_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->covariance != other.covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseWithCovariance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseWithCovariance_

// alias to use template instance with default allocator
using PoseWithCovariance =
  geometry_msgs::msg::PoseWithCovariance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/PoseWithCovarianceStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


#ifndef DIMOS_CDR_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B
#define DIMOS_CDR_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'pose'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__PoseWithCovarianceStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__PoseWithCovarianceStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PoseWithCovarianceStamped_
{
  using Type = PoseWithCovarianceStamped_<ContainerAllocator>;
void validate() const {
header.validate();
pose.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/PoseWithCovarianceStamped";

  explicit PoseWithCovarianceStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    (void)_init;
  }

  explicit PoseWithCovarianceStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__PoseWithCovarianceStamped
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__PoseWithCovarianceStamped
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseWithCovarianceStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseWithCovarianceStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseWithCovarianceStamped_

// alias to use template instance with default allocator
using PoseWithCovarianceStamped =
  geometry_msgs::msg::PoseWithCovarianceStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/QuaternionStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/quaternion_stamped.hpp"


#ifndef DIMOS_CDR_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5
#define DIMOS_CDR_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'quaternion'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__QuaternionStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__QuaternionStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct QuaternionStamped_
{
  using Type = QuaternionStamped_<ContainerAllocator>;
void validate() const {
header.validate();
quaternion.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/QuaternionStamped";

  explicit QuaternionStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    quaternion(_init)
  {
    (void)_init;
  }

  explicit QuaternionStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    quaternion(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _quaternion_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _quaternion_type quaternion;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__quaternion(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->quaternion = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::QuaternionStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::QuaternionStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::QuaternionStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::QuaternionStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::QuaternionStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::QuaternionStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::QuaternionStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::QuaternionStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::QuaternionStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::QuaternionStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__QuaternionStamped
    std::shared_ptr<geometry_msgs::msg::QuaternionStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__QuaternionStamped
    std::shared_ptr<geometry_msgs::msg::QuaternionStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QuaternionStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->quaternion != other.quaternion) {
      return false;
    }
    return true;
  }
  bool operator!=(const QuaternionStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QuaternionStamped_

// alias to use template instance with default allocator
using QuaternionStamped =
  geometry_msgs::msg::QuaternionStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Transform.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/transform.hpp"


#ifndef DIMOS_CDR_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238
#define DIMOS_CDR_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'translation'
// Member 'rotation'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Transform __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Transform __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Transform_
{
  using Type = Transform_<ContainerAllocator>;
void validate() const {
translation.validate();
rotation.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Transform";

  explicit Transform_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : translation(_init),
    rotation(_init)
  {
    (void)_init;
  }

  explicit Transform_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : translation(_alloc, _init),
    rotation(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _translation_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _translation_type translation;
  using _rotation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _rotation_type rotation;

  // setters for named parameter idiom
  Type & set__translation(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->translation = _arg;
    return *this;
  }
  Type & set__rotation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->rotation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Transform_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Transform_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Transform_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Transform_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Transform_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Transform_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Transform_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Transform_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Transform_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Transform_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Transform
    std::shared_ptr<geometry_msgs::msg::Transform_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Transform
    std::shared_ptr<geometry_msgs::msg::Transform_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Transform_ & other) const
  {
    if (this->translation != other.translation) {
      return false;
    }
    if (this->rotation != other.rotation) {
      return false;
    }
    return true;
  }
  bool operator!=(const Transform_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Transform_

// alias to use template instance with default allocator
using Transform =
  geometry_msgs::msg::Transform_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/TransformStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/transform_stamped.hpp"


#ifndef DIMOS_CDR_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA
#define DIMOS_CDR_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'transform'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__TransformStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__TransformStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TransformStamped_
{
  using Type = TransformStamped_<ContainerAllocator>;
void validate() const {
header.validate();
transform.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/TransformStamped";

  explicit TransformStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    transform(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->child_frame_id = "";
    }
  }

  explicit TransformStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    child_frame_id(_alloc),
    transform(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->child_frame_id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _child_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _child_frame_id_type child_frame_id;
  using _transform_type =
    geometry_msgs::msg::Transform_<ContainerAllocator>;
  _transform_type transform;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__child_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->child_frame_id = _arg;
    return *this;
  }
  Type & set__transform(
    const geometry_msgs::msg::Transform_<ContainerAllocator> & _arg)
  {
    this->transform = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::TransformStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::TransformStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::TransformStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::TransformStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::TransformStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::TransformStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::TransformStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::TransformStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::TransformStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::TransformStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__TransformStamped
    std::shared_ptr<geometry_msgs::msg::TransformStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__TransformStamped
    std::shared_ptr<geometry_msgs::msg::TransformStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TransformStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->child_frame_id != other.child_frame_id) {
      return false;
    }
    if (this->transform != other.transform) {
      return false;
    }
    return true;
  }
  bool operator!=(const TransformStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TransformStamped_

// alias to use template instance with default allocator
using TransformStamped =
  geometry_msgs::msg::TransformStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Twist.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/twist.hpp"


#ifndef DIMOS_CDR_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85
#define DIMOS_CDR_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'linear'
// Member 'angular'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Twist __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Twist __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Twist_
{
  using Type = Twist_<ContainerAllocator>;
void validate() const {
linear.validate();
angular.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Twist";

  explicit Twist_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : linear(_init),
    angular(_init)
  {
    (void)_init;
  }

  explicit Twist_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : linear(_alloc, _init),
    angular(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _linear_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_type linear;
  using _angular_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_type angular;

  // setters for named parameter idiom
  Type & set__linear(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear = _arg;
    return *this;
  }
  Type & set__angular(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Twist_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Twist_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Twist_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Twist_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Twist_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Twist_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Twist_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Twist_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Twist_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Twist_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Twist
    std::shared_ptr<geometry_msgs::msg::Twist_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Twist
    std::shared_ptr<geometry_msgs::msg::Twist_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Twist_ & other) const
  {
    if (this->linear != other.linear) {
      return false;
    }
    if (this->angular != other.angular) {
      return false;
    }
    return true;
  }
  bool operator!=(const Twist_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Twist_

// alias to use template instance with default allocator
using Twist =
  geometry_msgs::msg::Twist_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/TwistStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/twist_stamped.hpp"


#ifndef DIMOS_CDR_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C
#define DIMOS_CDR_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'twist'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__TwistStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__TwistStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TwistStamped_
{
  using Type = TwistStamped_<ContainerAllocator>;
void validate() const {
header.validate();
twist.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/TwistStamped";

  explicit TwistStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    twist(_init)
  {
    (void)_init;
  }

  explicit TwistStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    twist(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::TwistStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::TwistStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::TwistStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::TwistStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::TwistStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::TwistStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::TwistStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::TwistStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::TwistStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::TwistStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__TwistStamped
    std::shared_ptr<geometry_msgs::msg::TwistStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__TwistStamped
    std::shared_ptr<geometry_msgs::msg::TwistStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TwistStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const TwistStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TwistStamped_

// alias to use template instance with default allocator
using TwistStamped =
  geometry_msgs::msg::TwistStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/TwistWithCovariance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/twist_with_covariance.hpp"


#ifndef DIMOS_CDR_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD
#define DIMOS_CDR_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'twist'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__TwistWithCovariance __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__TwistWithCovariance __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TwistWithCovariance_
{
  using Type = TwistWithCovariance_<ContainerAllocator>;
void validate() const {
twist.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/TwistWithCovariance";

  explicit TwistWithCovariance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 36>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  explicit TwistWithCovariance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : twist(_alloc, _init),
    covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 36>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  // field types and members
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;
  using _covariance_type =
    std::array<double, 36>;
  _covariance_type covariance;

  // setters for named parameter idiom
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__covariance(
    const std::array<double, 36> & _arg)
  {
    this->covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__TwistWithCovariance
    std::shared_ptr<geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__TwistWithCovariance
    std::shared_ptr<geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TwistWithCovariance_ & other) const
  {
    if (this->twist != other.twist) {
      return false;
    }
    if (this->covariance != other.covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const TwistWithCovariance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TwistWithCovariance_

// alias to use template instance with default allocator
using TwistWithCovariance =
  geometry_msgs::msg::TwistWithCovariance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/TwistWithCovarianceStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"


#ifndef DIMOS_CDR_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6
#define DIMOS_CDR_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'twist'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__TwistWithCovarianceStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__TwistWithCovarianceStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TwistWithCovarianceStamped_
{
  using Type = TwistWithCovarianceStamped_<ContainerAllocator>;
void validate() const {
header.validate();
twist.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/TwistWithCovarianceStamped";

  explicit TwistWithCovarianceStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    twist(_init)
  {
    (void)_init;
  }

  explicit TwistWithCovarianceStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    twist(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _twist_type =
    geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>;
  _twist_type twist;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__TwistWithCovarianceStamped
    std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__TwistWithCovarianceStamped
    std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TwistWithCovarianceStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const TwistWithCovarianceStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TwistWithCovarianceStamped_

// alias to use template instance with default allocator
using TwistWithCovarianceStamped =
  geometry_msgs::msg::TwistWithCovarianceStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Vector3Stamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/vector3_stamped.hpp"


#ifndef DIMOS_CDR_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75
#define DIMOS_CDR_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'vector'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Vector3Stamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Vector3Stamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Vector3Stamped_
{
  using Type = Vector3Stamped_<ContainerAllocator>;
void validate() const {
header.validate();
vector.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Vector3Stamped";

  explicit Vector3Stamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    vector(_init)
  {
    (void)_init;
  }

  explicit Vector3Stamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    vector(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _vector_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _vector_type vector;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vector(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->vector = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Vector3Stamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Vector3Stamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Vector3Stamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Vector3Stamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Vector3Stamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Vector3Stamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Vector3Stamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Vector3Stamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Vector3Stamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Vector3Stamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Vector3Stamped
    std::shared_ptr<geometry_msgs::msg::Vector3Stamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Vector3Stamped
    std::shared_ptr<geometry_msgs::msg::Vector3Stamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Vector3Stamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vector != other.vector) {
      return false;
    }
    return true;
  }
  bool operator!=(const Vector3Stamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Vector3Stamped_

// alias to use template instance with default allocator
using Vector3Stamped =
  geometry_msgs::msg::Vector3Stamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/VelocityStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/velocity_stamped.hpp"


#ifndef DIMOS_CDR_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387
#define DIMOS_CDR_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'velocity'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__VelocityStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__VelocityStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelocityStamped_
{
  using Type = VelocityStamped_<ContainerAllocator>;
void validate() const {
header.validate();
velocity.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/VelocityStamped";

  explicit VelocityStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->body_frame_id = "";
      this->reference_frame_id = "";
    }
  }

  explicit VelocityStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    body_frame_id(_alloc),
    reference_frame_id(_alloc),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->body_frame_id = "";
      this->reference_frame_id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _body_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _body_frame_id_type body_frame_id;
  using _reference_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reference_frame_id_type reference_frame_id;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__body_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->body_frame_id = _arg;
    return *this;
  }
  Type & set__reference_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reference_frame_id = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::VelocityStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::VelocityStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::VelocityStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::VelocityStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::VelocityStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::VelocityStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::VelocityStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::VelocityStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::VelocityStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::VelocityStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__VelocityStamped
    std::shared_ptr<geometry_msgs::msg::VelocityStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__VelocityStamped
    std::shared_ptr<geometry_msgs::msg::VelocityStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelocityStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->body_frame_id != other.body_frame_id) {
      return false;
    }
    if (this->reference_frame_id != other.reference_frame_id) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelocityStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelocityStamped_

// alias to use template instance with default allocator
using VelocityStamped =
  geometry_msgs::msg::VelocityStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/VelocityWithCovarianceStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/velocity_with_covariance_stamped.hpp"


#ifndef DIMOS_CDR_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732
#define DIMOS_CDR_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'velocity'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__VelocityWithCovarianceStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__VelocityWithCovarianceStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelocityWithCovarianceStamped_
{
  using Type = VelocityWithCovarianceStamped_<ContainerAllocator>;
void validate() const {
header.validate();
velocity.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/VelocityWithCovarianceStamped";

  explicit VelocityWithCovarianceStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->body_frame_id = "";
      this->reference_frame_id = "";
    }
  }

  explicit VelocityWithCovarianceStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    body_frame_id(_alloc),
    reference_frame_id(_alloc),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->body_frame_id = "";
      this->reference_frame_id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _body_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _body_frame_id_type body_frame_id;
  using _reference_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reference_frame_id_type reference_frame_id;
  using _velocity_type =
    geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__body_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->body_frame_id = _arg;
    return *this;
  }
  Type & set__reference_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reference_frame_id = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__VelocityWithCovarianceStamped
    std::shared_ptr<geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__VelocityWithCovarianceStamped
    std::shared_ptr<geometry_msgs::msg::VelocityWithCovarianceStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelocityWithCovarianceStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->body_frame_id != other.body_frame_id) {
      return false;
    }
    if (this->reference_frame_id != other.reference_frame_id) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelocityWithCovarianceStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelocityWithCovarianceStamped_

// alias to use template instance with default allocator
using VelocityWithCovarianceStamped =
  geometry_msgs::msg::VelocityWithCovarianceStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/Wrench.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/wrench.hpp"


#ifndef DIMOS_CDR_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12
#define DIMOS_CDR_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'force'
// Member 'torque'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__Wrench __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__Wrench __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Wrench_
{
  using Type = Wrench_<ContainerAllocator>;
void validate() const {
force.validate();
torque.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/Wrench";

  explicit Wrench_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : force(_init),
    torque(_init)
  {
    (void)_init;
  }

  explicit Wrench_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : force(_alloc, _init),
    torque(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _force_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _force_type force;
  using _torque_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _torque_type torque;

  // setters for named parameter idiom
  Type & set__force(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->force = _arg;
    return *this;
  }
  Type & set__torque(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->torque = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::Wrench_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::Wrench_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::Wrench_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::Wrench_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Wrench_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Wrench_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::Wrench_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::Wrench_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::Wrench_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::Wrench_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__Wrench
    std::shared_ptr<geometry_msgs::msg::Wrench_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__Wrench
    std::shared_ptr<geometry_msgs::msg::Wrench_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Wrench_ & other) const
  {
    if (this->force != other.force) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    return true;
  }
  bool operator!=(const Wrench_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Wrench_

// alias to use template instance with default allocator
using Wrench =
  geometry_msgs::msg::Wrench_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from geometry_msgs:msg/WrenchStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "geometry_msgs/msg/wrench_stamped.hpp"


#ifndef DIMOS_CDR_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB
#define DIMOS_CDR_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'wrench'

#ifndef _WIN32
# define DEPRECATED__geometry_msgs__msg__WrenchStamped __attribute__((deprecated))
#else
# define DEPRECATED__geometry_msgs__msg__WrenchStamped __declspec(deprecated)
#endif

namespace geometry_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WrenchStamped_
{
  using Type = WrenchStamped_<ContainerAllocator>;
void validate() const {
header.validate();
wrench.validate();
}
static constexpr const char* msg_name = "geometry_msgs/msg/WrenchStamped";

  explicit WrenchStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    wrench(_init)
  {
    (void)_init;
  }

  explicit WrenchStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    wrench(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _wrench_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _wrench_type wrench;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__wrench(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->wrench = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    geometry_msgs::msg::WrenchStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const geometry_msgs::msg::WrenchStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<geometry_msgs::msg::WrenchStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<geometry_msgs::msg::WrenchStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::WrenchStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::WrenchStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      geometry_msgs::msg::WrenchStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<geometry_msgs::msg::WrenchStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<geometry_msgs::msg::WrenchStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<geometry_msgs::msg::WrenchStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__geometry_msgs__msg__WrenchStamped
    std::shared_ptr<geometry_msgs::msg::WrenchStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__geometry_msgs__msg__WrenchStamped
    std::shared_ptr<geometry_msgs::msg::WrenchStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WrenchStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->wrench != other.wrench) {
      return false;
    }
    return true;
  }
  bool operator!=(const WrenchStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WrenchStamped_

// alias to use template instance with default allocator
using WrenchStamped =
  geometry_msgs::msg::WrenchStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace geometry_msgs

#endif  // DIMOS_CDR_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav_msgs:msg/Goals.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_msgs/msg/goals.hpp"


#ifndef DIMOS_CDR_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0
#define DIMOS_CDR_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'goals'

#ifndef _WIN32
# define DEPRECATED__nav_msgs__msg__Goals __attribute__((deprecated))
#else
# define DEPRECATED__nav_msgs__msg__Goals __declspec(deprecated)
#endif

namespace nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Goals_
{
  using Type = Goals_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : goals) { item.validate(); }
}
static constexpr const char* msg_name = "nav_msgs/msg/Goals";

  explicit Goals_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Goals_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _goals_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _goals_type goals;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__goals(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->goals = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav_msgs::msg::Goals_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav_msgs::msg::Goals_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav_msgs::msg::Goals_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav_msgs::msg::Goals_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::Goals_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::Goals_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::Goals_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::Goals_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav_msgs::msg::Goals_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav_msgs::msg::Goals_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav_msgs__msg__Goals
    std::shared_ptr<nav_msgs::msg::Goals_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav_msgs__msg__Goals
    std::shared_ptr<nav_msgs::msg::Goals_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Goals_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->goals != other.goals) {
      return false;
    }
    return true;
  }
  bool operator!=(const Goals_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Goals_

// alias to use template instance with default allocator
using Goals =
  nav_msgs::msg::Goals_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav_msgs

#endif  // DIMOS_CDR_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav_msgs:msg/GridCells.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_msgs/msg/grid_cells.hpp"


#ifndef DIMOS_CDR_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B
#define DIMOS_CDR_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'cells'

#ifndef _WIN32
# define DEPRECATED__nav_msgs__msg__GridCells __attribute__((deprecated))
#else
# define DEPRECATED__nav_msgs__msg__GridCells __declspec(deprecated)
#endif

namespace nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GridCells_
{
  using Type = GridCells_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : cells) { item.validate(); }
}
static constexpr const char* msg_name = "nav_msgs/msg/GridCells";

  explicit GridCells_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cell_width = 0.0f;
      this->cell_height = 0.0f;
    }
  }

  explicit GridCells_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cell_width = 0.0f;
      this->cell_height = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _cell_width_type =
    float;
  _cell_width_type cell_width;
  using _cell_height_type =
    float;
  _cell_height_type cell_height;
  using _cells_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _cells_type cells;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__cell_width(
    const float & _arg)
  {
    this->cell_width = _arg;
    return *this;
  }
  Type & set__cell_height(
    const float & _arg)
  {
    this->cell_height = _arg;
    return *this;
  }
  Type & set__cells(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->cells = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav_msgs::msg::GridCells_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav_msgs::msg::GridCells_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav_msgs::msg::GridCells_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav_msgs::msg::GridCells_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::GridCells_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::GridCells_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::GridCells_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::GridCells_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav_msgs::msg::GridCells_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav_msgs::msg::GridCells_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav_msgs__msg__GridCells
    std::shared_ptr<nav_msgs::msg::GridCells_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav_msgs__msg__GridCells
    std::shared_ptr<nav_msgs::msg::GridCells_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GridCells_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->cell_width != other.cell_width) {
      return false;
    }
    if (this->cell_height != other.cell_height) {
      return false;
    }
    if (this->cells != other.cells) {
      return false;
    }
    return true;
  }
  bool operator!=(const GridCells_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GridCells_

// alias to use template instance with default allocator
using GridCells =
  nav_msgs::msg::GridCells_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav_msgs

#endif  // DIMOS_CDR_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav_msgs:msg/MapMetaData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_msgs/msg/map_meta_data.hpp"


#ifndef DIMOS_CDR_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0
#define DIMOS_CDR_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'map_load_time'
// Member 'origin'

#ifndef _WIN32
# define DEPRECATED__nav_msgs__msg__MapMetaData __attribute__((deprecated))
#else
# define DEPRECATED__nav_msgs__msg__MapMetaData __declspec(deprecated)
#endif

namespace nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MapMetaData_
{
  using Type = MapMetaData_<ContainerAllocator>;
void validate() const {
map_load_time.validate();
origin.validate();
}
static constexpr const char* msg_name = "nav_msgs/msg/MapMetaData";

  explicit MapMetaData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_load_time(_init),
    origin(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->resolution = 0.0f;
      this->width = 0ul;
      this->height = 0ul;
    }
  }

  explicit MapMetaData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_load_time(_alloc, _init),
    origin(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->resolution = 0.0f;
      this->width = 0ul;
      this->height = 0ul;
    }
  }

  // field types and members
  using _map_load_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _map_load_time_type map_load_time;
  using _resolution_type =
    float;
  _resolution_type resolution;
  using _width_type =
    uint32_t;
  _width_type width;
  using _height_type =
    uint32_t;
  _height_type height;
  using _origin_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _origin_type origin;

  // setters for named parameter idiom
  Type & set__map_load_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->map_load_time = _arg;
    return *this;
  }
  Type & set__resolution(
    const float & _arg)
  {
    this->resolution = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__origin(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->origin = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav_msgs::msg::MapMetaData_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav_msgs::msg::MapMetaData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav_msgs::msg::MapMetaData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav_msgs::msg::MapMetaData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::MapMetaData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::MapMetaData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::MapMetaData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::MapMetaData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav_msgs::msg::MapMetaData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav_msgs::msg::MapMetaData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav_msgs__msg__MapMetaData
    std::shared_ptr<nav_msgs::msg::MapMetaData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav_msgs__msg__MapMetaData
    std::shared_ptr<nav_msgs::msg::MapMetaData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapMetaData_ & other) const
  {
    if (this->map_load_time != other.map_load_time) {
      return false;
    }
    if (this->resolution != other.resolution) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->origin != other.origin) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapMetaData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapMetaData_

// alias to use template instance with default allocator
using MapMetaData =
  nav_msgs::msg::MapMetaData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav_msgs

#endif  // DIMOS_CDR_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav_msgs:msg/OccupancyGrid.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_msgs/msg/occupancy_grid.hpp"


#ifndef DIMOS_CDR_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D
#define DIMOS_CDR_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'info'

#ifndef _WIN32
# define DEPRECATED__nav_msgs__msg__OccupancyGrid __attribute__((deprecated))
#else
# define DEPRECATED__nav_msgs__msg__OccupancyGrid __declspec(deprecated)
#endif

namespace nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OccupancyGrid_
{
  using Type = OccupancyGrid_<ContainerAllocator>;
void validate() const {
header.validate();
info.validate();
}
static constexpr const char* msg_name = "nav_msgs/msg/OccupancyGrid";

  explicit OccupancyGrid_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    info(_init)
  {
    (void)_init;
  }

  explicit OccupancyGrid_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _info_type =
    nav_msgs::msg::MapMetaData_<ContainerAllocator>;
  _info_type info;
  using _data_type =
    std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__info(
    const nav_msgs::msg::MapMetaData_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav_msgs::msg::OccupancyGrid_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav_msgs::msg::OccupancyGrid_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav_msgs::msg::OccupancyGrid_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav_msgs::msg::OccupancyGrid_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::OccupancyGrid_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::OccupancyGrid_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::OccupancyGrid_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::OccupancyGrid_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav_msgs::msg::OccupancyGrid_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav_msgs::msg::OccupancyGrid_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav_msgs__msg__OccupancyGrid
    std::shared_ptr<nav_msgs::msg::OccupancyGrid_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav_msgs__msg__OccupancyGrid
    std::shared_ptr<nav_msgs::msg::OccupancyGrid_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OccupancyGrid_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->info != other.info) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const OccupancyGrid_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OccupancyGrid_

// alias to use template instance with default allocator
using OccupancyGrid =
  nav_msgs::msg::OccupancyGrid_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav_msgs

#endif  // DIMOS_CDR_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav_msgs:msg/Odometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_msgs/msg/odometry.hpp"


#ifndef DIMOS_CDR_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24
#define DIMOS_CDR_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'pose'
// Member 'twist'

#ifndef _WIN32
# define DEPRECATED__nav_msgs__msg__Odometry __attribute__((deprecated))
#else
# define DEPRECATED__nav_msgs__msg__Odometry __declspec(deprecated)
#endif

namespace nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Odometry_
{
  using Type = Odometry_<ContainerAllocator>;
void validate() const {
header.validate();
pose.validate();
twist.validate();
}
static constexpr const char* msg_name = "nav_msgs/msg/Odometry";

  explicit Odometry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->child_frame_id = "";
    }
  }

  explicit Odometry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    child_frame_id(_alloc),
    pose(_alloc, _init),
    twist(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->child_frame_id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _child_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _child_frame_id_type child_frame_id;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _pose_type pose;
  using _twist_type =
    geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator>;
  _twist_type twist;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__child_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->child_frame_id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::TwistWithCovariance_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav_msgs::msg::Odometry_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav_msgs::msg::Odometry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav_msgs::msg::Odometry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav_msgs::msg::Odometry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::Odometry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::Odometry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::Odometry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::Odometry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav_msgs::msg::Odometry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav_msgs::msg::Odometry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav_msgs__msg__Odometry
    std::shared_ptr<nav_msgs::msg::Odometry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav_msgs__msg__Odometry
    std::shared_ptr<nav_msgs::msg::Odometry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Odometry_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->child_frame_id != other.child_frame_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const Odometry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Odometry_

// alias to use template instance with default allocator
using Odometry =
  nav_msgs::msg::Odometry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav_msgs

#endif  // DIMOS_CDR_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav_msgs:msg/Path.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_msgs/msg/path.hpp"


#ifndef DIMOS_CDR_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7
#define DIMOS_CDR_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'poses'

#ifndef _WIN32
# define DEPRECATED__nav_msgs__msg__Path __attribute__((deprecated))
#else
# define DEPRECATED__nav_msgs__msg__Path __declspec(deprecated)
#endif

namespace nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Path_
{
  using Type = Path_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : poses) { item.validate(); }
}
static constexpr const char* msg_name = "nav_msgs/msg/Path";

  explicit Path_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Path_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _poses_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _poses_type poses;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__poses(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav_msgs::msg::Path_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav_msgs::msg::Path_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav_msgs::msg::Path_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav_msgs::msg::Path_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::Path_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::Path_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::Path_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::Path_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav_msgs::msg::Path_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav_msgs::msg::Path_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav_msgs__msg__Path
    std::shared_ptr<nav_msgs::msg::Path_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav_msgs__msg__Path
    std::shared_ptr<nav_msgs::msg::Path_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Path_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->poses != other.poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const Path_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Path_

// alias to use template instance with default allocator
using Path =
  nav_msgs::msg::Path_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav_msgs

#endif  // DIMOS_CDR_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav_msgs:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_msgs/msg/trajectory_point.hpp"


#ifndef DIMOS_CDR_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96
#define DIMOS_CDR_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'pose'
// Member 'velocity'
// Member 'acceleration'
// Member 'effort'

#ifndef _WIN32
# define DEPRECATED__nav_msgs__msg__TrajectoryPoint __attribute__((deprecated))
#else
# define DEPRECATED__nav_msgs__msg__TrajectoryPoint __declspec(deprecated)
#endif

namespace nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryPoint_
{
  using Type = TrajectoryPoint_<ContainerAllocator>;
void validate() const {
header.validate();
pose.validate();
velocity.validate();
acceleration.validate();
effort.validate();
}
static constexpr const char* msg_name = "nav_msgs/msg/TrajectoryPoint";

  explicit TrajectoryPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    velocity(_init),
    acceleration(_init),
    effort(_init)
  {
    (void)_init;
  }

  explicit TrajectoryPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    velocity(_alloc, _init),
    acceleration(_alloc, _init),
    effort(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _effort_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _effort_type effort;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__effort(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->effort = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav_msgs::msg::TrajectoryPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav_msgs::msg::TrajectoryPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav_msgs__msg__TrajectoryPoint
    std::shared_ptr<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav_msgs__msg__TrajectoryPoint
    std::shared_ptr<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryPoint_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryPoint_

// alias to use template instance with default allocator
using TrajectoryPoint =
  nav_msgs::msg::TrajectoryPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav_msgs

#endif  // DIMOS_CDR_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav_msgs/msg/trajectory.hpp"


#ifndef DIMOS_CDR_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA
#define DIMOS_CDR_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'points'

#ifndef _WIN32
# define DEPRECATED__nav_msgs__msg__Trajectory __attribute__((deprecated))
#else
# define DEPRECATED__nav_msgs__msg__Trajectory __declspec(deprecated)
#endif

namespace nav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Trajectory_
{
  using Type = Trajectory_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : points) { item.validate(); }
}
static constexpr const char* msg_name = "nav_msgs/msg/Trajectory";

  explicit Trajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Trajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _points_type =
    std::vector<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav_msgs::msg::TrajectoryPoint_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav_msgs::msg::Trajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav_msgs::msg::Trajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav_msgs::msg::Trajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav_msgs::msg::Trajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::Trajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::Trajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav_msgs::msg::Trajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav_msgs::msg::Trajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav_msgs::msg::Trajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav_msgs::msg::Trajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav_msgs__msg__Trajectory
    std::shared_ptr<nav_msgs::msg::Trajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav_msgs__msg__Trajectory
    std::shared_ptr<nav_msgs::msg::Trajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Trajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const Trajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Trajectory_

// alias to use template instance with default allocator
using Trajectory =
  nav_msgs::msg::Trajectory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav_msgs

#endif  // DIMOS_CDR_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/BatteryState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/battery_state.hpp"


#ifndef DIMOS_CDR_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5
#define DIMOS_CDR_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__BatteryState __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__BatteryState __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BatteryState_
{
  using Type = BatteryState_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/BatteryState";

  explicit BatteryState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0.0f;
      this->temperature = 0.0f;
      this->current = 0.0f;
      this->charge = 0.0f;
      this->capacity = 0.0f;
      this->design_capacity = 0.0f;
      this->percentage = 0.0f;
      this->power_supply_status = 0;
      this->power_supply_health = 0;
      this->power_supply_technology = 0;
      this->present = false;
      this->location = "";
      this->serial_number = "";
    }
  }

  explicit BatteryState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    location(_alloc),
    serial_number(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0.0f;
      this->temperature = 0.0f;
      this->current = 0.0f;
      this->charge = 0.0f;
      this->capacity = 0.0f;
      this->design_capacity = 0.0f;
      this->percentage = 0.0f;
      this->power_supply_status = 0;
      this->power_supply_health = 0;
      this->power_supply_technology = 0;
      this->present = false;
      this->location = "";
      this->serial_number = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _voltage_type =
    float;
  _voltage_type voltage;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _current_type =
    float;
  _current_type current;
  using _charge_type =
    float;
  _charge_type charge;
  using _capacity_type =
    float;
  _capacity_type capacity;
  using _design_capacity_type =
    float;
  _design_capacity_type design_capacity;
  using _percentage_type =
    float;
  _percentage_type percentage;
  using _power_supply_status_type =
    uint8_t;
  _power_supply_status_type power_supply_status;
  using _power_supply_health_type =
    uint8_t;
  _power_supply_health_type power_supply_health;
  using _power_supply_technology_type =
    uint8_t;
  _power_supply_technology_type power_supply_technology;
  using _present_type =
    bool;
  _present_type present;
  using _cell_voltage_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _cell_voltage_type cell_voltage;
  using _cell_temperature_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _cell_temperature_type cell_temperature;
  using _location_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _location_type location;
  using _serial_number_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _serial_number_type serial_number;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__voltage(
    const float & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__current(
    const float & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__charge(
    const float & _arg)
  {
    this->charge = _arg;
    return *this;
  }
  Type & set__capacity(
    const float & _arg)
  {
    this->capacity = _arg;
    return *this;
  }
  Type & set__design_capacity(
    const float & _arg)
  {
    this->design_capacity = _arg;
    return *this;
  }
  Type & set__percentage(
    const float & _arg)
  {
    this->percentage = _arg;
    return *this;
  }
  Type & set__power_supply_status(
    const uint8_t & _arg)
  {
    this->power_supply_status = _arg;
    return *this;
  }
  Type & set__power_supply_health(
    const uint8_t & _arg)
  {
    this->power_supply_health = _arg;
    return *this;
  }
  Type & set__power_supply_technology(
    const uint8_t & _arg)
  {
    this->power_supply_technology = _arg;
    return *this;
  }
  Type & set__present(
    const bool & _arg)
  {
    this->present = _arg;
    return *this;
  }
  Type & set__cell_voltage(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->cell_voltage = _arg;
    return *this;
  }
  Type & set__cell_temperature(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->cell_temperature = _arg;
    return *this;
  }
  Type & set__location(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->location = _arg;
    return *this;
  }
  Type & set__serial_number(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->serial_number = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t POWER_SUPPLY_STATUS_UNKNOWN =
    0u;
  static constexpr uint8_t POWER_SUPPLY_STATUS_CHARGING =
    1u;
  static constexpr uint8_t POWER_SUPPLY_STATUS_DISCHARGING =
    2u;
  static constexpr uint8_t POWER_SUPPLY_STATUS_NOT_CHARGING =
    3u;
  static constexpr uint8_t POWER_SUPPLY_STATUS_FULL =
    4u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_UNKNOWN =
    0u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_GOOD =
    1u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERHEAT =
    2u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_DEAD =
    3u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_OVERVOLTAGE =
    4u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_UNSPEC_FAILURE =
    5u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_COLD =
    6u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE =
    7u;
  static constexpr uint8_t POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE =
    8u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_UNKNOWN =
    0u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NIMH =
    1u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LION =
    2u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIPO =
    3u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIFE =
    4u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_NICD =
    5u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_LIMN =
    6u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_TERNARY =
    7u;
  static constexpr uint8_t POWER_SUPPLY_TECHNOLOGY_VRLA =
    8u;

  // pointer types
  using RawPtr =
    sensor_msgs::msg::BatteryState_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::BatteryState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::BatteryState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::BatteryState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::BatteryState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::BatteryState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::BatteryState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::BatteryState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::BatteryState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::BatteryState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__BatteryState
    std::shared_ptr<sensor_msgs::msg::BatteryState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__BatteryState
    std::shared_ptr<sensor_msgs::msg::BatteryState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BatteryState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->charge != other.charge) {
      return false;
    }
    if (this->capacity != other.capacity) {
      return false;
    }
    if (this->design_capacity != other.design_capacity) {
      return false;
    }
    if (this->percentage != other.percentage) {
      return false;
    }
    if (this->power_supply_status != other.power_supply_status) {
      return false;
    }
    if (this->power_supply_health != other.power_supply_health) {
      return false;
    }
    if (this->power_supply_technology != other.power_supply_technology) {
      return false;
    }
    if (this->present != other.present) {
      return false;
    }
    if (this->cell_voltage != other.cell_voltage) {
      return false;
    }
    if (this->cell_temperature != other.cell_temperature) {
      return false;
    }
    if (this->location != other.location) {
      return false;
    }
    if (this->serial_number != other.serial_number) {
      return false;
    }
    return true;
  }
  bool operator!=(const BatteryState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BatteryState_

// alias to use template instance with default allocator
using BatteryState =
  sensor_msgs::msg::BatteryState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_STATUS_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_STATUS_CHARGING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_STATUS_DISCHARGING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_STATUS_FULL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_GOOD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_OVERHEAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_DEAD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_COLD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_NIMH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_LION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_LIPO;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_LIFE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_NICD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_LIMN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_TERNARY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t BatteryState_<ContainerAllocator>::POWER_SUPPLY_TECHNOLOGY_VRLA;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/RegionOfInterest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/region_of_interest.hpp"


#ifndef DIMOS_CDR_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62
#define DIMOS_CDR_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__RegionOfInterest __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__RegionOfInterest __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RegionOfInterest_
{
  using Type = RegionOfInterest_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "sensor_msgs/msg/RegionOfInterest";

  explicit RegionOfInterest_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_offset = 0ul;
      this->y_offset = 0ul;
      this->height = 0ul;
      this->width = 0ul;
      this->do_rectify = false;
    }
  }

  explicit RegionOfInterest_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_offset = 0ul;
      this->y_offset = 0ul;
      this->height = 0ul;
      this->width = 0ul;
      this->do_rectify = false;
    }
  }

  // field types and members
  using _x_offset_type =
    uint32_t;
  _x_offset_type x_offset;
  using _y_offset_type =
    uint32_t;
  _y_offset_type y_offset;
  using _height_type =
    uint32_t;
  _height_type height;
  using _width_type =
    uint32_t;
  _width_type width;
  using _do_rectify_type =
    bool;
  _do_rectify_type do_rectify;

  // setters for named parameter idiom
  Type & set__x_offset(
    const uint32_t & _arg)
  {
    this->x_offset = _arg;
    return *this;
  }
  Type & set__y_offset(
    const uint32_t & _arg)
  {
    this->y_offset = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__do_rectify(
    const bool & _arg)
  {
    this->do_rectify = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__RegionOfInterest
    std::shared_ptr<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__RegionOfInterest
    std::shared_ptr<sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RegionOfInterest_ & other) const
  {
    if (this->x_offset != other.x_offset) {
      return false;
    }
    if (this->y_offset != other.y_offset) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->do_rectify != other.do_rectify) {
      return false;
    }
    return true;
  }
  bool operator!=(const RegionOfInterest_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RegionOfInterest_

// alias to use template instance with default allocator
using RegionOfInterest =
  sensor_msgs::msg::RegionOfInterest_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/CameraInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/camera_info.hpp"


#ifndef DIMOS_CDR_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5
#define DIMOS_CDR_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'roi'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__CameraInfo __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__CameraInfo __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraInfo_
{
  using Type = CameraInfo_<ContainerAllocator>;
void validate() const {
header.validate();
roi.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/CameraInfo";

  explicit CameraInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    roi(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0ul;
      this->width = 0ul;
      this->distortion_model = "";
      std::fill<typename std::array<double, 9>::iterator, double>(this->k.begin(), this->k.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->r.begin(), this->r.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->p.begin(), this->p.end(), 0.0);
      this->binning_x = 0ul;
      this->binning_y = 0ul;
    }
  }

  explicit CameraInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    distortion_model(_alloc),
    k(_alloc),
    r(_alloc),
    p(_alloc),
    roi(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0ul;
      this->width = 0ul;
      this->distortion_model = "";
      std::fill<typename std::array<double, 9>::iterator, double>(this->k.begin(), this->k.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->r.begin(), this->r.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->p.begin(), this->p.end(), 0.0);
      this->binning_x = 0ul;
      this->binning_y = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _height_type =
    uint32_t;
  _height_type height;
  using _width_type =
    uint32_t;
  _width_type width;
  using _distortion_model_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _distortion_model_type distortion_model;
  using _d_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _d_type d;
  using _k_type =
    std::array<double, 9>;
  _k_type k;
  using _r_type =
    std::array<double, 9>;
  _r_type r;
  using _p_type =
    std::array<double, 12>;
  _p_type p;
  using _binning_x_type =
    uint32_t;
  _binning_x_type binning_x;
  using _binning_y_type =
    uint32_t;
  _binning_y_type binning_y;
  using _roi_type =
    sensor_msgs::msg::RegionOfInterest_<ContainerAllocator>;
  _roi_type roi;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__distortion_model(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->distortion_model = _arg;
    return *this;
  }
  Type & set__d(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->d = _arg;
    return *this;
  }
  Type & set__k(
    const std::array<double, 9> & _arg)
  {
    this->k = _arg;
    return *this;
  }
  Type & set__r(
    const std::array<double, 9> & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__p(
    const std::array<double, 12> & _arg)
  {
    this->p = _arg;
    return *this;
  }
  Type & set__binning_x(
    const uint32_t & _arg)
  {
    this->binning_x = _arg;
    return *this;
  }
  Type & set__binning_y(
    const uint32_t & _arg)
  {
    this->binning_y = _arg;
    return *this;
  }
  Type & set__roi(
    const sensor_msgs::msg::RegionOfInterest_<ContainerAllocator> & _arg)
  {
    this->roi = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::CameraInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::CameraInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::CameraInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::CameraInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::CameraInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::CameraInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::CameraInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::CameraInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::CameraInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::CameraInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__CameraInfo
    std::shared_ptr<sensor_msgs::msg::CameraInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__CameraInfo
    std::shared_ptr<sensor_msgs::msg::CameraInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->distortion_model != other.distortion_model) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    if (this->k != other.k) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->p != other.p) {
      return false;
    }
    if (this->binning_x != other.binning_x) {
      return false;
    }
    if (this->binning_y != other.binning_y) {
      return false;
    }
    if (this->roi != other.roi) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraInfo_

// alias to use template instance with default allocator
using CameraInfo =
  sensor_msgs::msg::CameraInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/ChannelFloat32.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/channel_float32.hpp"


#ifndef DIMOS_CDR_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6
#define DIMOS_CDR_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__ChannelFloat32 __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__ChannelFloat32 __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChannelFloat32_
{
  using Type = ChannelFloat32_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "sensor_msgs/msg/ChannelFloat32";

  explicit ChannelFloat32_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  explicit ChannelFloat32_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _values_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _values_type values;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__values(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->values = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::ChannelFloat32_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::ChannelFloat32_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__ChannelFloat32
    std::shared_ptr<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__ChannelFloat32
    std::shared_ptr<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChannelFloat32_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->values != other.values) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChannelFloat32_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChannelFloat32_

// alias to use template instance with default allocator
using ChannelFloat32 =
  sensor_msgs::msg::ChannelFloat32_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/CompressedImage.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/compressed_image.hpp"


#ifndef DIMOS_CDR_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A
#define DIMOS_CDR_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__CompressedImage __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__CompressedImage __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CompressedImage_
{
  using Type = CompressedImage_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/CompressedImage";

  explicit CompressedImage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->format = "";
    }
  }

  explicit CompressedImage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    format(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->format = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _format_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _format_type format;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__format(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->format = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::CompressedImage_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::CompressedImage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::CompressedImage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::CompressedImage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::CompressedImage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::CompressedImage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::CompressedImage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::CompressedImage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::CompressedImage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::CompressedImage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__CompressedImage
    std::shared_ptr<sensor_msgs::msg::CompressedImage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__CompressedImage
    std::shared_ptr<sensor_msgs::msg::CompressedImage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CompressedImage_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->format != other.format) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const CompressedImage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CompressedImage_

// alias to use template instance with default allocator
using CompressedImage =
  sensor_msgs::msg::CompressedImage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/FluidPressure.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/fluid_pressure.hpp"


#ifndef DIMOS_CDR_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87
#define DIMOS_CDR_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__FluidPressure __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__FluidPressure __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FluidPressure_
{
  using Type = FluidPressure_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/FluidPressure";

  explicit FluidPressure_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fluid_pressure = 0.0;
      this->variance = 0.0;
    }
  }

  explicit FluidPressure_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fluid_pressure = 0.0;
      this->variance = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fluid_pressure_type =
    double;
  _fluid_pressure_type fluid_pressure;
  using _variance_type =
    double;
  _variance_type variance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__fluid_pressure(
    const double & _arg)
  {
    this->fluid_pressure = _arg;
    return *this;
  }
  Type & set__variance(
    const double & _arg)
  {
    this->variance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::FluidPressure_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::FluidPressure_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::FluidPressure_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::FluidPressure_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::FluidPressure_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::FluidPressure_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::FluidPressure_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::FluidPressure_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::FluidPressure_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::FluidPressure_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__FluidPressure
    std::shared_ptr<sensor_msgs::msg::FluidPressure_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__FluidPressure
    std::shared_ptr<sensor_msgs::msg::FluidPressure_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FluidPressure_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fluid_pressure != other.fluid_pressure) {
      return false;
    }
    if (this->variance != other.variance) {
      return false;
    }
    return true;
  }
  bool operator!=(const FluidPressure_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FluidPressure_

// alias to use template instance with default allocator
using FluidPressure =
  sensor_msgs::msg::FluidPressure_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/Illuminance.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/illuminance.hpp"


#ifndef DIMOS_CDR_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733
#define DIMOS_CDR_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__Illuminance __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__Illuminance __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Illuminance_
{
  using Type = Illuminance_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Illuminance";

  explicit Illuminance_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->illuminance = 0.0;
      this->variance = 0.0;
    }
  }

  explicit Illuminance_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->illuminance = 0.0;
      this->variance = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _illuminance_type =
    double;
  _illuminance_type illuminance;
  using _variance_type =
    double;
  _variance_type variance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__illuminance(
    const double & _arg)
  {
    this->illuminance = _arg;
    return *this;
  }
  Type & set__variance(
    const double & _arg)
  {
    this->variance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::Illuminance_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::Illuminance_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::Illuminance_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::Illuminance_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Illuminance_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Illuminance_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Illuminance_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Illuminance_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::Illuminance_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::Illuminance_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__Illuminance
    std::shared_ptr<sensor_msgs::msg::Illuminance_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__Illuminance
    std::shared_ptr<sensor_msgs::msg::Illuminance_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Illuminance_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->illuminance != other.illuminance) {
      return false;
    }
    if (this->variance != other.variance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Illuminance_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Illuminance_

// alias to use template instance with default allocator
using Illuminance =
  sensor_msgs::msg::Illuminance_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/Image.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/image.hpp"


#ifndef DIMOS_CDR_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25
#define DIMOS_CDR_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__Image __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__Image __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Image_
{
  using Type = Image_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Image";

  explicit Image_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0ul;
      this->width = 0ul;
      this->encoding = "";
      this->is_bigendian = 0;
      this->step = 0ul;
    }
  }

  explicit Image_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    encoding(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0ul;
      this->width = 0ul;
      this->encoding = "";
      this->is_bigendian = 0;
      this->step = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _height_type =
    uint32_t;
  _height_type height;
  using _width_type =
    uint32_t;
  _width_type width;
  using _encoding_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _encoding_type encoding;
  using _is_bigendian_type =
    uint8_t;
  _is_bigendian_type is_bigendian;
  using _step_type =
    uint32_t;
  _step_type step;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__encoding(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->encoding = _arg;
    return *this;
  }
  Type & set__is_bigendian(
    const uint8_t & _arg)
  {
    this->is_bigendian = _arg;
    return *this;
  }
  Type & set__step(
    const uint32_t & _arg)
  {
    this->step = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::Image_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::Image_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::Image_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::Image_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Image_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Image_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Image_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Image_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::Image_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::Image_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__Image
    std::shared_ptr<sensor_msgs::msg::Image_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__Image
    std::shared_ptr<sensor_msgs::msg::Image_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Image_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->encoding != other.encoding) {
      return false;
    }
    if (this->is_bigendian != other.is_bigendian) {
      return false;
    }
    if (this->step != other.step) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Image_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Image_

// alias to use template instance with default allocator
using Image =
  sensor_msgs::msg::Image_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/Imu.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/imu.hpp"


#ifndef DIMOS_CDR_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75
#define DIMOS_CDR_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'orientation'
// Member 'angular_velocity'
// Member 'linear_acceleration'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__Imu __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__Imu __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Imu_
{
  using Type = Imu_<ContainerAllocator>;
void validate() const {
header.validate();
orientation.validate();
angular_velocity.validate();
linear_acceleration.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Imu";

  explicit Imu_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    orientation(_init),
    angular_velocity(_init),
    linear_acceleration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->orientation_covariance.begin(), this->orientation_covariance.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->angular_velocity_covariance.begin(), this->angular_velocity_covariance.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->linear_acceleration_covariance.begin(), this->linear_acceleration_covariance.end(), 0.0);
    }
  }

  explicit Imu_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    orientation(_alloc, _init),
    orientation_covariance(_alloc),
    angular_velocity(_alloc, _init),
    angular_velocity_covariance(_alloc),
    linear_acceleration(_alloc, _init),
    linear_acceleration_covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->orientation_covariance.begin(), this->orientation_covariance.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->angular_velocity_covariance.begin(), this->angular_velocity_covariance.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->linear_acceleration_covariance.begin(), this->linear_acceleration_covariance.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _orientation_covariance_type =
    std::array<double, 9>;
  _orientation_covariance_type orientation_covariance;
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;
  using _angular_velocity_covariance_type =
    std::array<double, 9>;
  _angular_velocity_covariance_type angular_velocity_covariance;
  using _linear_acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_acceleration_type linear_acceleration;
  using _linear_acceleration_covariance_type =
    std::array<double, 9>;
  _linear_acceleration_covariance_type linear_acceleration_covariance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__orientation_covariance(
    const std::array<double, 9> & _arg)
  {
    this->orientation_covariance = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity_covariance(
    const std::array<double, 9> & _arg)
  {
    this->angular_velocity_covariance = _arg;
    return *this;
  }
  Type & set__linear_acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear_acceleration = _arg;
    return *this;
  }
  Type & set__linear_acceleration_covariance(
    const std::array<double, 9> & _arg)
  {
    this->linear_acceleration_covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::Imu_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::Imu_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::Imu_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::Imu_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Imu_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Imu_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Imu_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Imu_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::Imu_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::Imu_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__Imu
    std::shared_ptr<sensor_msgs::msg::Imu_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__Imu
    std::shared_ptr<sensor_msgs::msg::Imu_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Imu_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->orientation_covariance != other.orientation_covariance) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->angular_velocity_covariance != other.angular_velocity_covariance) {
      return false;
    }
    if (this->linear_acceleration != other.linear_acceleration) {
      return false;
    }
    if (this->linear_acceleration_covariance != other.linear_acceleration_covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Imu_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Imu_

// alias to use template instance with default allocator
using Imu =
  sensor_msgs::msg::Imu_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/JointState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/joint_state.hpp"


#ifndef DIMOS_CDR_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752
#define DIMOS_CDR_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__JointState __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__JointState __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointState_
{
  using Type = JointState_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/JointState";

  explicit JointState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit JointState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _name_type name;
  using _position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _position_type position;
  using _velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _velocity_type velocity;
  using _effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _effort_type effort;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__name(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->effort = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::JointState_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::JointState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::JointState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::JointState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::JointState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::JointState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::JointState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::JointState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::JointState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::JointState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__JointState
    std::shared_ptr<sensor_msgs::msg::JointState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__JointState
    std::shared_ptr<sensor_msgs::msg::JointState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointState_

// alias to use template instance with default allocator
using JointState =
  sensor_msgs::msg::JointState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/Joy.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/joy.hpp"


#ifndef DIMOS_CDR_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8
#define DIMOS_CDR_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__Joy __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__Joy __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Joy_
{
  using Type = Joy_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Joy";

  explicit Joy_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Joy_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _axes_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _axes_type axes;
  using _buttons_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _buttons_type buttons;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__axes(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->axes = _arg;
    return *this;
  }
  Type & set__buttons(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->buttons = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::Joy_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::Joy_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::Joy_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::Joy_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Joy_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Joy_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Joy_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Joy_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::Joy_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::Joy_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__Joy
    std::shared_ptr<sensor_msgs::msg::Joy_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__Joy
    std::shared_ptr<sensor_msgs::msg::Joy_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Joy_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->axes != other.axes) {
      return false;
    }
    if (this->buttons != other.buttons) {
      return false;
    }
    return true;
  }
  bool operator!=(const Joy_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Joy_

// alias to use template instance with default allocator
using Joy =
  sensor_msgs::msg::Joy_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/JoyFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/joy_feedback.hpp"


#ifndef DIMOS_CDR_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F
#define DIMOS_CDR_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__JoyFeedback __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__JoyFeedback __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JoyFeedback_
{
  using Type = JoyFeedback_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "sensor_msgs/msg/JoyFeedback";

  explicit JoyFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->id = 0;
      this->intensity = 0.0f;
    }
  }

  explicit JoyFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->id = 0;
      this->intensity = 0.0f;
    }
  }

  // field types and members
  using _type_type =
    uint8_t;
  _type_type type;
  using _id_type =
    uint8_t;
  _id_type id;
  using _intensity_type =
    float;
  _intensity_type intensity;

  // setters for named parameter idiom
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__intensity(
    const float & _arg)
  {
    this->intensity = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t TYPE_LED =
    0u;
  static constexpr uint8_t TYPE_RUMBLE =
    1u;
  static constexpr uint8_t TYPE_BUZZER =
    2u;

  // pointer types
  using RawPtr =
    sensor_msgs::msg::JoyFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::JoyFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::JoyFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::JoyFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::JoyFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::JoyFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::JoyFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::JoyFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::JoyFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::JoyFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__JoyFeedback
    std::shared_ptr<sensor_msgs::msg::JoyFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__JoyFeedback
    std::shared_ptr<sensor_msgs::msg::JoyFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JoyFeedback_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->intensity != other.intensity) {
      return false;
    }
    return true;
  }
  bool operator!=(const JoyFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JoyFeedback_

// alias to use template instance with default allocator
using JoyFeedback =
  sensor_msgs::msg::JoyFeedback_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyFeedback_<ContainerAllocator>::TYPE_LED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyFeedback_<ContainerAllocator>::TYPE_RUMBLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t JoyFeedback_<ContainerAllocator>::TYPE_BUZZER;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/JoyFeedbackArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/joy_feedback_array.hpp"


#ifndef DIMOS_CDR_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF
#define DIMOS_CDR_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'array'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__JoyFeedbackArray __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__JoyFeedbackArray __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JoyFeedbackArray_
{
  using Type = JoyFeedbackArray_<ContainerAllocator>;
void validate() const {
for (const auto& item : array) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/JoyFeedbackArray";

  explicit JoyFeedbackArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit JoyFeedbackArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _array_type =
    std::vector<sensor_msgs::msg::JoyFeedback_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::JoyFeedback_<ContainerAllocator>>>;
  _array_type array;

  // setters for named parameter idiom
  Type & set__array(
    const std::vector<sensor_msgs::msg::JoyFeedback_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::JoyFeedback_<ContainerAllocator>>> & _arg)
  {
    this->array = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__JoyFeedbackArray
    std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__JoyFeedbackArray
    std::shared_ptr<sensor_msgs::msg::JoyFeedbackArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JoyFeedbackArray_ & other) const
  {
    if (this->array != other.array) {
      return false;
    }
    return true;
  }
  bool operator!=(const JoyFeedbackArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JoyFeedbackArray_

// alias to use template instance with default allocator
using JoyFeedbackArray =
  sensor_msgs::msg::JoyFeedbackArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/LaserEcho.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/laser_echo.hpp"


#ifndef DIMOS_CDR_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71
#define DIMOS_CDR_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__LaserEcho __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__LaserEcho __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LaserEcho_
{
  using Type = LaserEcho_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "sensor_msgs/msg/LaserEcho";

  explicit LaserEcho_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit LaserEcho_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _echoes_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _echoes_type echoes;

  // setters for named parameter idiom
  Type & set__echoes(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->echoes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::LaserEcho_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::LaserEcho_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::LaserEcho_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::LaserEcho_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::LaserEcho_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::LaserEcho_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::LaserEcho_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::LaserEcho_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::LaserEcho_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::LaserEcho_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__LaserEcho
    std::shared_ptr<sensor_msgs::msg::LaserEcho_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__LaserEcho
    std::shared_ptr<sensor_msgs::msg::LaserEcho_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaserEcho_ & other) const
  {
    if (this->echoes != other.echoes) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaserEcho_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaserEcho_

// alias to use template instance with default allocator
using LaserEcho =
  sensor_msgs::msg::LaserEcho_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/LaserScan.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/laser_scan.hpp"


#ifndef DIMOS_CDR_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419
#define DIMOS_CDR_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__LaserScan __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__LaserScan __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LaserScan_
{
  using Type = LaserScan_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/LaserScan";

  explicit LaserScan_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_min = 0.0f;
      this->angle_max = 0.0f;
      this->angle_increment = 0.0f;
      this->time_increment = 0.0f;
      this->scan_time = 0.0f;
      this->range_min = 0.0f;
      this->range_max = 0.0f;
    }
  }

  explicit LaserScan_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_min = 0.0f;
      this->angle_max = 0.0f;
      this->angle_increment = 0.0f;
      this->time_increment = 0.0f;
      this->scan_time = 0.0f;
      this->range_min = 0.0f;
      this->range_max = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _angle_min_type =
    float;
  _angle_min_type angle_min;
  using _angle_max_type =
    float;
  _angle_max_type angle_max;
  using _angle_increment_type =
    float;
  _angle_increment_type angle_increment;
  using _time_increment_type =
    float;
  _time_increment_type time_increment;
  using _scan_time_type =
    float;
  _scan_time_type scan_time;
  using _range_min_type =
    float;
  _range_min_type range_min;
  using _range_max_type =
    float;
  _range_max_type range_max;
  using _ranges_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _ranges_type ranges;
  using _intensities_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _intensities_type intensities;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__angle_min(
    const float & _arg)
  {
    this->angle_min = _arg;
    return *this;
  }
  Type & set__angle_max(
    const float & _arg)
  {
    this->angle_max = _arg;
    return *this;
  }
  Type & set__angle_increment(
    const float & _arg)
  {
    this->angle_increment = _arg;
    return *this;
  }
  Type & set__time_increment(
    const float & _arg)
  {
    this->time_increment = _arg;
    return *this;
  }
  Type & set__scan_time(
    const float & _arg)
  {
    this->scan_time = _arg;
    return *this;
  }
  Type & set__range_min(
    const float & _arg)
  {
    this->range_min = _arg;
    return *this;
  }
  Type & set__range_max(
    const float & _arg)
  {
    this->range_max = _arg;
    return *this;
  }
  Type & set__ranges(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->ranges = _arg;
    return *this;
  }
  Type & set__intensities(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->intensities = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::LaserScan_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::LaserScan_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::LaserScan_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::LaserScan_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::LaserScan_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::LaserScan_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::LaserScan_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::LaserScan_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::LaserScan_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::LaserScan_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__LaserScan
    std::shared_ptr<sensor_msgs::msg::LaserScan_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__LaserScan
    std::shared_ptr<sensor_msgs::msg::LaserScan_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaserScan_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->angle_min != other.angle_min) {
      return false;
    }
    if (this->angle_max != other.angle_max) {
      return false;
    }
    if (this->angle_increment != other.angle_increment) {
      return false;
    }
    if (this->time_increment != other.time_increment) {
      return false;
    }
    if (this->scan_time != other.scan_time) {
      return false;
    }
    if (this->range_min != other.range_min) {
      return false;
    }
    if (this->range_max != other.range_max) {
      return false;
    }
    if (this->ranges != other.ranges) {
      return false;
    }
    if (this->intensities != other.intensities) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaserScan_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaserScan_

// alias to use template instance with default allocator
using LaserScan =
  sensor_msgs::msg::LaserScan_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/MagneticField.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/magnetic_field.hpp"


#ifndef DIMOS_CDR_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E
#define DIMOS_CDR_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'magnetic_field'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__MagneticField __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__MagneticField __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MagneticField_
{
  using Type = MagneticField_<ContainerAllocator>;
void validate() const {
header.validate();
magnetic_field.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/MagneticField";

  explicit MagneticField_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    magnetic_field(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->magnetic_field_covariance.begin(), this->magnetic_field_covariance.end(), 0.0);
    }
  }

  explicit MagneticField_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    magnetic_field(_alloc, _init),
    magnetic_field_covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 9>::iterator, double>(this->magnetic_field_covariance.begin(), this->magnetic_field_covariance.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _magnetic_field_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _magnetic_field_type magnetic_field;
  using _magnetic_field_covariance_type =
    std::array<double, 9>;
  _magnetic_field_covariance_type magnetic_field_covariance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__magnetic_field(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->magnetic_field = _arg;
    return *this;
  }
  Type & set__magnetic_field_covariance(
    const std::array<double, 9> & _arg)
  {
    this->magnetic_field_covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::MagneticField_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::MagneticField_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::MagneticField_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::MagneticField_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::MagneticField_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::MagneticField_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::MagneticField_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::MagneticField_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::MagneticField_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::MagneticField_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__MagneticField
    std::shared_ptr<sensor_msgs::msg::MagneticField_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__MagneticField
    std::shared_ptr<sensor_msgs::msg::MagneticField_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MagneticField_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->magnetic_field != other.magnetic_field) {
      return false;
    }
    if (this->magnetic_field_covariance != other.magnetic_field_covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const MagneticField_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MagneticField_

// alias to use template instance with default allocator
using MagneticField =
  sensor_msgs::msg::MagneticField_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/MultiDOFJointState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/multi_dof_joint_state.hpp"


#ifndef DIMOS_CDR_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C
#define DIMOS_CDR_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'transforms'
// Member 'twist'
// Member 'wrench'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__MultiDOFJointState __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__MultiDOFJointState __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiDOFJointState_
{
  using Type = MultiDOFJointState_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : transforms) { item.validate(); }
for (const auto& item : twist) { item.validate(); }
for (const auto& item : wrench) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/MultiDOFJointState";

  explicit MultiDOFJointState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MultiDOFJointState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _joint_names_type joint_names;
  using _transforms_type =
    std::vector<geometry_msgs::msg::Transform_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Transform_<ContainerAllocator>>>;
  _transforms_type transforms;
  using _twist_type =
    std::vector<geometry_msgs::msg::Twist_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Twist_<ContainerAllocator>>>;
  _twist_type twist;
  using _wrench_type =
    std::vector<geometry_msgs::msg::Wrench_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Wrench_<ContainerAllocator>>>;
  _wrench_type wrench;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->joint_names = _arg;
    return *this;
  }
  Type & set__transforms(
    const std::vector<geometry_msgs::msg::Transform_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Transform_<ContainerAllocator>>> & _arg)
  {
    this->transforms = _arg;
    return *this;
  }
  Type & set__twist(
    const std::vector<geometry_msgs::msg::Twist_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Twist_<ContainerAllocator>>> & _arg)
  {
    this->twist = _arg;
    return *this;
  }
  Type & set__wrench(
    const std::vector<geometry_msgs::msg::Wrench_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Wrench_<ContainerAllocator>>> & _arg)
  {
    this->wrench = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__MultiDOFJointState
    std::shared_ptr<sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__MultiDOFJointState
    std::shared_ptr<sensor_msgs::msg::MultiDOFJointState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiDOFJointState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_names != other.joint_names) {
      return false;
    }
    if (this->transforms != other.transforms) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    if (this->wrench != other.wrench) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiDOFJointState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiDOFJointState_

// alias to use template instance with default allocator
using MultiDOFJointState =
  sensor_msgs::msg::MultiDOFJointState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/MultiEchoLaserScan.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/multi_echo_laser_scan.hpp"


#ifndef DIMOS_CDR_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF
#define DIMOS_CDR_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'ranges'
// Member 'intensities'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__MultiEchoLaserScan __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__MultiEchoLaserScan __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiEchoLaserScan_
{
  using Type = MultiEchoLaserScan_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : ranges) { item.validate(); }
for (const auto& item : intensities) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/MultiEchoLaserScan";

  explicit MultiEchoLaserScan_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_min = 0.0f;
      this->angle_max = 0.0f;
      this->angle_increment = 0.0f;
      this->time_increment = 0.0f;
      this->scan_time = 0.0f;
      this->range_min = 0.0f;
      this->range_max = 0.0f;
    }
  }

  explicit MultiEchoLaserScan_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle_min = 0.0f;
      this->angle_max = 0.0f;
      this->angle_increment = 0.0f;
      this->time_increment = 0.0f;
      this->scan_time = 0.0f;
      this->range_min = 0.0f;
      this->range_max = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _angle_min_type =
    float;
  _angle_min_type angle_min;
  using _angle_max_type =
    float;
  _angle_max_type angle_max;
  using _angle_increment_type =
    float;
  _angle_increment_type angle_increment;
  using _time_increment_type =
    float;
  _time_increment_type time_increment;
  using _scan_time_type =
    float;
  _scan_time_type scan_time;
  using _range_min_type =
    float;
  _range_min_type range_min;
  using _range_max_type =
    float;
  _range_max_type range_max;
  using _ranges_type =
    std::vector<sensor_msgs::msg::LaserEcho_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::LaserEcho_<ContainerAllocator>>>;
  _ranges_type ranges;
  using _intensities_type =
    std::vector<sensor_msgs::msg::LaserEcho_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::LaserEcho_<ContainerAllocator>>>;
  _intensities_type intensities;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__angle_min(
    const float & _arg)
  {
    this->angle_min = _arg;
    return *this;
  }
  Type & set__angle_max(
    const float & _arg)
  {
    this->angle_max = _arg;
    return *this;
  }
  Type & set__angle_increment(
    const float & _arg)
  {
    this->angle_increment = _arg;
    return *this;
  }
  Type & set__time_increment(
    const float & _arg)
  {
    this->time_increment = _arg;
    return *this;
  }
  Type & set__scan_time(
    const float & _arg)
  {
    this->scan_time = _arg;
    return *this;
  }
  Type & set__range_min(
    const float & _arg)
  {
    this->range_min = _arg;
    return *this;
  }
  Type & set__range_max(
    const float & _arg)
  {
    this->range_max = _arg;
    return *this;
  }
  Type & set__ranges(
    const std::vector<sensor_msgs::msg::LaserEcho_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::LaserEcho_<ContainerAllocator>>> & _arg)
  {
    this->ranges = _arg;
    return *this;
  }
  Type & set__intensities(
    const std::vector<sensor_msgs::msg::LaserEcho_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::LaserEcho_<ContainerAllocator>>> & _arg)
  {
    this->intensities = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__MultiEchoLaserScan
    std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__MultiEchoLaserScan
    std::shared_ptr<sensor_msgs::msg::MultiEchoLaserScan_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiEchoLaserScan_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->angle_min != other.angle_min) {
      return false;
    }
    if (this->angle_max != other.angle_max) {
      return false;
    }
    if (this->angle_increment != other.angle_increment) {
      return false;
    }
    if (this->time_increment != other.time_increment) {
      return false;
    }
    if (this->scan_time != other.scan_time) {
      return false;
    }
    if (this->range_min != other.range_min) {
      return false;
    }
    if (this->range_max != other.range_max) {
      return false;
    }
    if (this->ranges != other.ranges) {
      return false;
    }
    if (this->intensities != other.intensities) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiEchoLaserScan_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiEchoLaserScan_

// alias to use template instance with default allocator
using MultiEchoLaserScan =
  sensor_msgs::msg::MultiEchoLaserScan_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/NavSatStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/nav_sat_status.hpp"


#ifndef DIMOS_CDR_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2
#define DIMOS_CDR_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__NavSatStatus __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__NavSatStatus __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavSatStatus_
{
  using Type = NavSatStatus_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "sensor_msgs/msg/NavSatStatus";

  explicit NavSatStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->status = -2;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->status = 0;
      this->service = 0;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->service = 0;
    }
  }

  explicit NavSatStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->status = -2;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->status = 0;
      this->service = 0;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->service = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _service_type =
    uint16_t;
  _service_type service;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__service(
    const uint16_t & _arg)
  {
    this->service = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t STATUS_UNKNOWN =
    -2;
  static constexpr int8_t STATUS_NO_FIX =
    -1;
  static constexpr int8_t STATUS_FIX =
    0;
  static constexpr int8_t STATUS_SBAS_FIX =
    1;
  static constexpr int8_t STATUS_GBAS_FIX =
    2;
  static constexpr uint16_t SERVICE_UNKNOWN =
    0u;
  static constexpr uint16_t SERVICE_GPS =
    1u;
  static constexpr uint16_t SERVICE_GLONASS =
    2u;
  static constexpr uint16_t SERVICE_COMPASS =
    4u;
  static constexpr uint16_t SERVICE_GALILEO =
    8u;

  // pointer types
  using RawPtr =
    sensor_msgs::msg::NavSatStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::NavSatStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::NavSatStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::NavSatStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::NavSatStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::NavSatStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::NavSatStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::NavSatStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::NavSatStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::NavSatStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__NavSatStatus
    std::shared_ptr<sensor_msgs::msg::NavSatStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__NavSatStatus
    std::shared_ptr<sensor_msgs::msg::NavSatStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavSatStatus_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->service != other.service) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavSatStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavSatStatus_

// alias to use template instance with default allocator
using NavSatStatus =
  sensor_msgs::msg::NavSatStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t NavSatStatus_<ContainerAllocator>::STATUS_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t NavSatStatus_<ContainerAllocator>::STATUS_NO_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t NavSatStatus_<ContainerAllocator>::STATUS_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t NavSatStatus_<ContainerAllocator>::STATUS_SBAS_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t NavSatStatus_<ContainerAllocator>::STATUS_GBAS_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t NavSatStatus_<ContainerAllocator>::SERVICE_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t NavSatStatus_<ContainerAllocator>::SERVICE_GPS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t NavSatStatus_<ContainerAllocator>::SERVICE_GLONASS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t NavSatStatus_<ContainerAllocator>::SERVICE_COMPASS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t NavSatStatus_<ContainerAllocator>::SERVICE_GALILEO;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/NavSatFix.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/nav_sat_fix.hpp"


#ifndef DIMOS_CDR_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9
#define DIMOS_CDR_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'status'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__NavSatFix __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__NavSatFix __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavSatFix_
{
  using Type = NavSatFix_<ContainerAllocator>;
void validate() const {
header.validate();
status.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/NavSatFix";

  explicit NavSatFix_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    status(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      std::fill<typename std::array<double, 9>::iterator, double>(this->position_covariance.begin(), this->position_covariance.end(), 0.0);
      this->position_covariance_type = 0;
    }
  }

  explicit NavSatFix_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    status(_alloc, _init),
    position_covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      std::fill<typename std::array<double, 9>::iterator, double>(this->position_covariance.begin(), this->position_covariance.end(), 0.0);
      this->position_covariance_type = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_type =
    sensor_msgs::msg::NavSatStatus_<ContainerAllocator>;
  _status_type status;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _position_covariance_type =
    std::array<double, 9>;
  _position_covariance_type position_covariance;
  using _position_covariance_type_type =
    uint8_t;
  _position_covariance_type_type position_covariance_type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status(
    const sensor_msgs::msg::NavSatStatus_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__position_covariance(
    const std::array<double, 9> & _arg)
  {
    this->position_covariance = _arg;
    return *this;
  }
  Type & set__position_covariance_type(
    const uint8_t & _arg)
  {
    this->position_covariance_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t COVARIANCE_TYPE_UNKNOWN =
    0u;
  static constexpr uint8_t COVARIANCE_TYPE_APPROXIMATED =
    1u;
  static constexpr uint8_t COVARIANCE_TYPE_DIAGONAL_KNOWN =
    2u;
  static constexpr uint8_t COVARIANCE_TYPE_KNOWN =
    3u;

  // pointer types
  using RawPtr =
    sensor_msgs::msg::NavSatFix_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::NavSatFix_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::NavSatFix_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::NavSatFix_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::NavSatFix_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::NavSatFix_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::NavSatFix_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::NavSatFix_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::NavSatFix_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::NavSatFix_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__NavSatFix
    std::shared_ptr<sensor_msgs::msg::NavSatFix_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__NavSatFix
    std::shared_ptr<sensor_msgs::msg::NavSatFix_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavSatFix_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->position_covariance != other.position_covariance) {
      return false;
    }
    if (this->position_covariance_type != other.position_covariance_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavSatFix_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavSatFix_

// alias to use template instance with default allocator
using NavSatFix =
  sensor_msgs::msg::NavSatFix_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NavSatFix_<ContainerAllocator>::COVARIANCE_TYPE_UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NavSatFix_<ContainerAllocator>::COVARIANCE_TYPE_APPROXIMATED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NavSatFix_<ContainerAllocator>::COVARIANCE_TYPE_DIAGONAL_KNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NavSatFix_<ContainerAllocator>::COVARIANCE_TYPE_KNOWN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/PointCloud.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/point_cloud.hpp"


#ifndef DIMOS_CDR_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D
#define DIMOS_CDR_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'points'
// Member 'channels'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__PointCloud __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__PointCloud __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointCloud_
{
  using Type = PointCloud_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : points) { item.validate(); }
for (const auto& item : channels) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/PointCloud";

  explicit PointCloud_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit PointCloud_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _points_type =
    std::vector<geometry_msgs::msg::Point32_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point32_<ContainerAllocator>>>;
  _points_type points;
  using _channels_type =
    std::vector<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>>>;
  _channels_type channels;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point32_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point32_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__channels(
    const std::vector<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::ChannelFloat32_<ContainerAllocator>>> & _arg)
  {
    this->channels = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::PointCloud_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::PointCloud_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::PointCloud_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::PointCloud_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::PointCloud_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::PointCloud_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::PointCloud_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::PointCloud_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::PointCloud_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::PointCloud_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__PointCloud
    std::shared_ptr<sensor_msgs::msg::PointCloud_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__PointCloud
    std::shared_ptr<sensor_msgs::msg::PointCloud_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointCloud_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->channels != other.channels) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointCloud_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointCloud_

// alias to use template instance with default allocator
using PointCloud =
  sensor_msgs::msg::PointCloud_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/PointField.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/point_field.hpp"


#ifndef DIMOS_CDR_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018
#define DIMOS_CDR_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__PointField __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__PointField __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointField_
{
  using Type = PointField_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "sensor_msgs/msg/PointField";

  explicit PointField_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->offset = 0ul;
      this->datatype = 0;
      this->count = 0ul;
    }
  }

  explicit PointField_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->offset = 0ul;
      this->datatype = 0;
      this->count = 0ul;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _offset_type =
    uint32_t;
  _offset_type offset;
  using _datatype_type =
    uint8_t;
  _datatype_type datatype;
  using _count_type =
    uint32_t;
  _count_type count;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__offset(
    const uint32_t & _arg)
  {
    this->offset = _arg;
    return *this;
  }
  Type & set__datatype(
    const uint8_t & _arg)
  {
    this->datatype = _arg;
    return *this;
  }
  Type & set__count(
    const uint32_t & _arg)
  {
    this->count = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t INT8 =
    1u;
  static constexpr uint8_t UINT8 =
    2u;
  static constexpr uint8_t INT16 =
    3u;
  static constexpr uint8_t UINT16 =
    4u;
  static constexpr uint8_t INT32 =
    5u;
  static constexpr uint8_t UINT32 =
    6u;
  static constexpr uint8_t FLOAT32 =
    7u;
  static constexpr uint8_t FLOAT64 =
    8u;

  // pointer types
  using RawPtr =
    sensor_msgs::msg::PointField_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::PointField_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::PointField_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::PointField_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::PointField_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::PointField_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::PointField_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::PointField_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::PointField_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::PointField_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__PointField
    std::shared_ptr<sensor_msgs::msg::PointField_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__PointField
    std::shared_ptr<sensor_msgs::msg::PointField_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointField_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->offset != other.offset) {
      return false;
    }
    if (this->datatype != other.datatype) {
      return false;
    }
    if (this->count != other.count) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointField_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointField_

// alias to use template instance with default allocator
using PointField =
  sensor_msgs::msg::PointField_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PointField_<ContainerAllocator>::INT8;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PointField_<ContainerAllocator>::UINT8;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PointField_<ContainerAllocator>::INT16;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PointField_<ContainerAllocator>::UINT16;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PointField_<ContainerAllocator>::INT32;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PointField_<ContainerAllocator>::UINT32;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PointField_<ContainerAllocator>::FLOAT32;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PointField_<ContainerAllocator>::FLOAT64;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/PointCloud2.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/point_cloud2.hpp"


#ifndef DIMOS_CDR_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973
#define DIMOS_CDR_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'fields'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__PointCloud2 __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__PointCloud2 __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointCloud2_
{
  using Type = PointCloud2_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : fields) { item.validate(); }
}
static constexpr const char* msg_name = "sensor_msgs/msg/PointCloud2";

  explicit PointCloud2_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0ul;
      this->width = 0ul;
      this->is_bigendian = false;
      this->point_step = 0ul;
      this->row_step = 0ul;
      this->is_dense = false;
    }
  }

  explicit PointCloud2_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0ul;
      this->width = 0ul;
      this->is_bigendian = false;
      this->point_step = 0ul;
      this->row_step = 0ul;
      this->is_dense = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _height_type =
    uint32_t;
  _height_type height;
  using _width_type =
    uint32_t;
  _width_type width;
  using _fields_type =
    std::vector<sensor_msgs::msg::PointField_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::PointField_<ContainerAllocator>>>;
  _fields_type fields;
  using _is_bigendian_type =
    bool;
  _is_bigendian_type is_bigendian;
  using _point_step_type =
    uint32_t;
  _point_step_type point_step;
  using _row_step_type =
    uint32_t;
  _row_step_type row_step;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;
  using _is_dense_type =
    bool;
  _is_dense_type is_dense;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__fields(
    const std::vector<sensor_msgs::msg::PointField_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::PointField_<ContainerAllocator>>> & _arg)
  {
    this->fields = _arg;
    return *this;
  }
  Type & set__is_bigendian(
    const bool & _arg)
  {
    this->is_bigendian = _arg;
    return *this;
  }
  Type & set__point_step(
    const uint32_t & _arg)
  {
    this->point_step = _arg;
    return *this;
  }
  Type & set__row_step(
    const uint32_t & _arg)
  {
    this->row_step = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__is_dense(
    const bool & _arg)
  {
    this->is_dense = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::PointCloud2_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::PointCloud2_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::PointCloud2_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::PointCloud2_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::PointCloud2_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::PointCloud2_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__PointCloud2
    std::shared_ptr<sensor_msgs::msg::PointCloud2_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__PointCloud2
    std::shared_ptr<sensor_msgs::msg::PointCloud2_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointCloud2_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->fields != other.fields) {
      return false;
    }
    if (this->is_bigendian != other.is_bigendian) {
      return false;
    }
    if (this->point_step != other.point_step) {
      return false;
    }
    if (this->row_step != other.row_step) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->is_dense != other.is_dense) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointCloud2_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointCloud2_

// alias to use template instance with default allocator
using PointCloud2 =
  sensor_msgs::msg::PointCloud2_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/Range.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/range.hpp"


#ifndef DIMOS_CDR_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579
#define DIMOS_CDR_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__Range __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__Range __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Range_
{
  using Type = Range_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Range";

  explicit Range_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->radiation_type = 0;
      this->field_of_view = 0.0f;
      this->min_range = 0.0f;
      this->max_range = 0.0f;
      this->range = 0.0f;
      this->variance = 0.0f;
    }
  }

  explicit Range_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->radiation_type = 0;
      this->field_of_view = 0.0f;
      this->min_range = 0.0f;
      this->max_range = 0.0f;
      this->range = 0.0f;
      this->variance = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _radiation_type_type =
    uint8_t;
  _radiation_type_type radiation_type;
  using _field_of_view_type =
    float;
  _field_of_view_type field_of_view;
  using _min_range_type =
    float;
  _min_range_type min_range;
  using _max_range_type =
    float;
  _max_range_type max_range;
  using _range_type =
    float;
  _range_type range;
  using _variance_type =
    float;
  _variance_type variance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__radiation_type(
    const uint8_t & _arg)
  {
    this->radiation_type = _arg;
    return *this;
  }
  Type & set__field_of_view(
    const float & _arg)
  {
    this->field_of_view = _arg;
    return *this;
  }
  Type & set__min_range(
    const float & _arg)
  {
    this->min_range = _arg;
    return *this;
  }
  Type & set__max_range(
    const float & _arg)
  {
    this->max_range = _arg;
    return *this;
  }
  Type & set__range(
    const float & _arg)
  {
    this->range = _arg;
    return *this;
  }
  Type & set__variance(
    const float & _arg)
  {
    this->variance = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t ULTRASOUND =
    0u;
  static constexpr uint8_t INFRARED =
    1u;

  // pointer types
  using RawPtr =
    sensor_msgs::msg::Range_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::Range_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::Range_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::Range_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Range_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Range_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Range_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Range_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::Range_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::Range_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__Range
    std::shared_ptr<sensor_msgs::msg::Range_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__Range
    std::shared_ptr<sensor_msgs::msg::Range_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Range_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->radiation_type != other.radiation_type) {
      return false;
    }
    if (this->field_of_view != other.field_of_view) {
      return false;
    }
    if (this->min_range != other.min_range) {
      return false;
    }
    if (this->max_range != other.max_range) {
      return false;
    }
    if (this->range != other.range) {
      return false;
    }
    if (this->variance != other.variance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Range_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Range_

// alias to use template instance with default allocator
using Range =
  sensor_msgs::msg::Range_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Range_<ContainerAllocator>::ULTRASOUND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Range_<ContainerAllocator>::INFRARED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/RelativeHumidity.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/relative_humidity.hpp"


#ifndef DIMOS_CDR_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0
#define DIMOS_CDR_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__RelativeHumidity __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__RelativeHumidity __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RelativeHumidity_
{
  using Type = RelativeHumidity_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/RelativeHumidity";

  explicit RelativeHumidity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->relative_humidity = 0.0;
      this->variance = 0.0;
    }
  }

  explicit RelativeHumidity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->relative_humidity = 0.0;
      this->variance = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _relative_humidity_type =
    double;
  _relative_humidity_type relative_humidity;
  using _variance_type =
    double;
  _variance_type variance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__relative_humidity(
    const double & _arg)
  {
    this->relative_humidity = _arg;
    return *this;
  }
  Type & set__variance(
    const double & _arg)
  {
    this->variance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::RelativeHumidity_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::RelativeHumidity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::RelativeHumidity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::RelativeHumidity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::RelativeHumidity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::RelativeHumidity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::RelativeHumidity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::RelativeHumidity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::RelativeHumidity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::RelativeHumidity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__RelativeHumidity
    std::shared_ptr<sensor_msgs::msg::RelativeHumidity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__RelativeHumidity
    std::shared_ptr<sensor_msgs::msg::RelativeHumidity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RelativeHumidity_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->relative_humidity != other.relative_humidity) {
      return false;
    }
    if (this->variance != other.variance) {
      return false;
    }
    return true;
  }
  bool operator!=(const RelativeHumidity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RelativeHumidity_

// alias to use template instance with default allocator
using RelativeHumidity =
  sensor_msgs::msg::RelativeHumidity_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/Temperature.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/temperature.hpp"


#ifndef DIMOS_CDR_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D
#define DIMOS_CDR_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__Temperature __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__Temperature __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Temperature_
{
  using Type = Temperature_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/Temperature";

  explicit Temperature_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->temperature = 0.0;
      this->variance = 0.0;
    }
  }

  explicit Temperature_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->temperature = 0.0;
      this->variance = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _temperature_type =
    double;
  _temperature_type temperature;
  using _variance_type =
    double;
  _variance_type variance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__temperature(
    const double & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__variance(
    const double & _arg)
  {
    this->variance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::Temperature_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::Temperature_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::Temperature_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::Temperature_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Temperature_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Temperature_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::Temperature_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::Temperature_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::Temperature_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::Temperature_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__Temperature
    std::shared_ptr<sensor_msgs::msg::Temperature_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__Temperature
    std::shared_ptr<sensor_msgs::msg::Temperature_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Temperature_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->variance != other.variance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Temperature_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Temperature_

// alias to use template instance with default allocator
using Temperature =
  sensor_msgs::msg::Temperature_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor_msgs:msg/TimeReference.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sensor_msgs/msg/time_reference.hpp"


#ifndef DIMOS_CDR_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430
#define DIMOS_CDR_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'time_ref'

#ifndef _WIN32
# define DEPRECATED__sensor_msgs__msg__TimeReference __attribute__((deprecated))
#else
# define DEPRECATED__sensor_msgs__msg__TimeReference __declspec(deprecated)
#endif

namespace sensor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TimeReference_
{
  using Type = TimeReference_<ContainerAllocator>;
void validate() const {
header.validate();
time_ref.validate();
}
static constexpr const char* msg_name = "sensor_msgs/msg/TimeReference";

  explicit TimeReference_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    time_ref(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->source = "";
    }
  }

  explicit TimeReference_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    time_ref(_alloc, _init),
    source(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->source = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_ref_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _time_ref_type time_ref;
  using _source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _source_type source;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__time_ref(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->time_ref = _arg;
    return *this;
  }
  Type & set__source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->source = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor_msgs::msg::TimeReference_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor_msgs::msg::TimeReference_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor_msgs::msg::TimeReference_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor_msgs::msg::TimeReference_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::TimeReference_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::TimeReference_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor_msgs::msg::TimeReference_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor_msgs::msg::TimeReference_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor_msgs::msg::TimeReference_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor_msgs::msg::TimeReference_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor_msgs__msg__TimeReference
    std::shared_ptr<sensor_msgs::msg::TimeReference_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor_msgs__msg__TimeReference
    std::shared_ptr<sensor_msgs::msg::TimeReference_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimeReference_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time_ref != other.time_ref) {
      return false;
    }
    if (this->source != other.source) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimeReference_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimeReference_

// alias to use template instance with default allocator
using TimeReference =
  sensor_msgs::msg::TimeReference_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor_msgs

#endif  // DIMOS_CDR_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shape_msgs:msg/MeshTriangle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shape_msgs/msg/mesh_triangle.hpp"


#ifndef DIMOS_CDR_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50
#define DIMOS_CDR_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__shape_msgs__msg__MeshTriangle __attribute__((deprecated))
#else
# define DEPRECATED__shape_msgs__msg__MeshTriangle __declspec(deprecated)
#endif

namespace shape_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshTriangle_
{
  using Type = MeshTriangle_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "shape_msgs/msg/MeshTriangle";

  explicit MeshTriangle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->vertex_indices.begin(), this->vertex_indices.end(), 0ul);
    }
  }

  explicit MeshTriangle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vertex_indices(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->vertex_indices.begin(), this->vertex_indices.end(), 0ul);
    }
  }

  // field types and members
  using _vertex_indices_type =
    std::array<uint32_t, 3>;
  _vertex_indices_type vertex_indices;

  // setters for named parameter idiom
  Type & set__vertex_indices(
    const std::array<uint32_t, 3> & _arg)
  {
    this->vertex_indices = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shape_msgs::msg::MeshTriangle_<ContainerAllocator> *;
  using ConstRawPtr =
    const shape_msgs::msg::MeshTriangle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shape_msgs::msg::MeshTriangle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shape_msgs::msg::MeshTriangle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shape_msgs::msg::MeshTriangle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shape_msgs::msg::MeshTriangle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shape_msgs::msg::MeshTriangle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shape_msgs::msg::MeshTriangle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shape_msgs::msg::MeshTriangle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shape_msgs::msg::MeshTriangle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shape_msgs__msg__MeshTriangle
    std::shared_ptr<shape_msgs::msg::MeshTriangle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shape_msgs__msg__MeshTriangle
    std::shared_ptr<shape_msgs::msg::MeshTriangle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshTriangle_ & other) const
  {
    if (this->vertex_indices != other.vertex_indices) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshTriangle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshTriangle_

// alias to use template instance with default allocator
using MeshTriangle =
  shape_msgs::msg::MeshTriangle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace shape_msgs

#endif  // DIMOS_CDR_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shape_msgs:msg/Mesh.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shape_msgs/msg/mesh.hpp"


#ifndef DIMOS_CDR_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68
#define DIMOS_CDR_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'triangles'
// Member 'vertices'

#ifndef _WIN32
# define DEPRECATED__shape_msgs__msg__Mesh __attribute__((deprecated))
#else
# define DEPRECATED__shape_msgs__msg__Mesh __declspec(deprecated)
#endif

namespace shape_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Mesh_
{
  using Type = Mesh_<ContainerAllocator>;
void validate() const {
for (const auto& item : triangles) { item.validate(); }
for (const auto& item : vertices) { item.validate(); }
}
static constexpr const char* msg_name = "shape_msgs/msg/Mesh";

  explicit Mesh_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Mesh_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _triangles_type =
    std::vector<shape_msgs::msg::MeshTriangle_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<shape_msgs::msg::MeshTriangle_<ContainerAllocator>>>;
  _triangles_type triangles;
  using _vertices_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _vertices_type vertices;

  // setters for named parameter idiom
  Type & set__triangles(
    const std::vector<shape_msgs::msg::MeshTriangle_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<shape_msgs::msg::MeshTriangle_<ContainerAllocator>>> & _arg)
  {
    this->triangles = _arg;
    return *this;
  }
  Type & set__vertices(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->vertices = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shape_msgs::msg::Mesh_<ContainerAllocator> *;
  using ConstRawPtr =
    const shape_msgs::msg::Mesh_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shape_msgs::msg::Mesh_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shape_msgs::msg::Mesh_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shape_msgs::msg::Mesh_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shape_msgs::msg::Mesh_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shape_msgs::msg::Mesh_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shape_msgs::msg::Mesh_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shape_msgs::msg::Mesh_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shape_msgs::msg::Mesh_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shape_msgs__msg__Mesh
    std::shared_ptr<shape_msgs::msg::Mesh_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shape_msgs__msg__Mesh
    std::shared_ptr<shape_msgs::msg::Mesh_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Mesh_ & other) const
  {
    if (this->triangles != other.triangles) {
      return false;
    }
    if (this->vertices != other.vertices) {
      return false;
    }
    return true;
  }
  bool operator!=(const Mesh_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Mesh_

// alias to use template instance with default allocator
using Mesh =
  shape_msgs::msg::Mesh_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace shape_msgs

#endif  // DIMOS_CDR_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shape_msgs:msg/Plane.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shape_msgs/msg/plane.hpp"


#ifndef DIMOS_CDR_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5
#define DIMOS_CDR_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__shape_msgs__msg__Plane __attribute__((deprecated))
#else
# define DEPRECATED__shape_msgs__msg__Plane __declspec(deprecated)
#endif

namespace shape_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Plane_
{
  using Type = Plane_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "shape_msgs/msg/Plane";

  explicit Plane_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 4>::iterator, double>(this->coef.begin(), this->coef.end(), 0.0);
    }
  }

  explicit Plane_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : coef(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 4>::iterator, double>(this->coef.begin(), this->coef.end(), 0.0);
    }
  }

  // field types and members
  using _coef_type =
    std::array<double, 4>;
  _coef_type coef;

  // setters for named parameter idiom
  Type & set__coef(
    const std::array<double, 4> & _arg)
  {
    this->coef = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shape_msgs::msg::Plane_<ContainerAllocator> *;
  using ConstRawPtr =
    const shape_msgs::msg::Plane_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shape_msgs::msg::Plane_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shape_msgs::msg::Plane_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shape_msgs::msg::Plane_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shape_msgs::msg::Plane_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shape_msgs::msg::Plane_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shape_msgs::msg::Plane_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shape_msgs::msg::Plane_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shape_msgs::msg::Plane_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shape_msgs__msg__Plane
    std::shared_ptr<shape_msgs::msg::Plane_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shape_msgs__msg__Plane
    std::shared_ptr<shape_msgs::msg::Plane_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Plane_ & other) const
  {
    if (this->coef != other.coef) {
      return false;
    }
    return true;
  }
  bool operator!=(const Plane_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Plane_

// alias to use template instance with default allocator
using Plane =
  shape_msgs::msg::Plane_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace shape_msgs

#endif  // DIMOS_CDR_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shape_msgs:msg/SolidPrimitive.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "shape_msgs/msg/solid_primitive.hpp"


#ifndef DIMOS_CDR_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5
#define DIMOS_CDR_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'polygon'

#ifndef _WIN32
# define DEPRECATED__shape_msgs__msg__SolidPrimitive __attribute__((deprecated))
#else
# define DEPRECATED__shape_msgs__msg__SolidPrimitive __declspec(deprecated)
#endif

namespace shape_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SolidPrimitive_
{
  using Type = SolidPrimitive_<ContainerAllocator>;
void validate() const {
if (dimensions.size() > 3) throw std::length_error("dimensions exceeds sequence bound");
polygon.validate();
}
static constexpr const char* msg_name = "shape_msgs/msg/SolidPrimitive";

  explicit SolidPrimitive_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : polygon(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
    }
  }

  explicit SolidPrimitive_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : polygon(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
    }
  }

  // field types and members
  using _type_type =
    uint8_t;
  _type_type type;
  using _dimensions_type =
    rosidl_runtime_cpp::BoundedVector<double, 3, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _dimensions_type dimensions;
  using _polygon_type =
    geometry_msgs::msg::Polygon_<ContainerAllocator>;
  _polygon_type polygon;

  // setters for named parameter idiom
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__dimensions(
    const rosidl_runtime_cpp::BoundedVector<double, 3, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->dimensions = _arg;
    return *this;
  }
  Type & set__polygon(
    const geometry_msgs::msg::Polygon_<ContainerAllocator> & _arg)
  {
    this->polygon = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t BOX =
    1u;
  static constexpr uint8_t SPHERE =
    2u;
  static constexpr uint8_t CYLINDER =
    3u;
  static constexpr uint8_t CONE =
    4u;
  static constexpr uint8_t PRISM =
    5u;
  static constexpr uint8_t BOX_X =
    0u;
  static constexpr uint8_t BOX_Y =
    1u;
  static constexpr uint8_t BOX_Z =
    2u;
  static constexpr uint8_t SPHERE_RADIUS =
    0u;
  static constexpr uint8_t CYLINDER_HEIGHT =
    0u;
  static constexpr uint8_t CYLINDER_RADIUS =
    1u;
  static constexpr uint8_t CONE_HEIGHT =
    0u;
  static constexpr uint8_t CONE_RADIUS =
    1u;
  static constexpr uint8_t PRISM_HEIGHT =
    0u;

  // pointer types
  using RawPtr =
    shape_msgs::msg::SolidPrimitive_<ContainerAllocator> *;
  using ConstRawPtr =
    const shape_msgs::msg::SolidPrimitive_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shape_msgs::msg::SolidPrimitive_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shape_msgs::msg::SolidPrimitive_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shape_msgs::msg::SolidPrimitive_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shape_msgs::msg::SolidPrimitive_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shape_msgs::msg::SolidPrimitive_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shape_msgs::msg::SolidPrimitive_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shape_msgs::msg::SolidPrimitive_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shape_msgs::msg::SolidPrimitive_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shape_msgs__msg__SolidPrimitive
    std::shared_ptr<shape_msgs::msg::SolidPrimitive_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shape_msgs__msg__SolidPrimitive
    std::shared_ptr<shape_msgs::msg::SolidPrimitive_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SolidPrimitive_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->dimensions != other.dimensions) {
      return false;
    }
    if (this->polygon != other.polygon) {
      return false;
    }
    return true;
  }
  bool operator!=(const SolidPrimitive_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SolidPrimitive_

// alias to use template instance with default allocator
using SolidPrimitive =
  shape_msgs::msg::SolidPrimitive_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::BOX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::SPHERE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::CYLINDER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::CONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::PRISM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::BOX_X;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::BOX_Y;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::BOX_Z;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::SPHERE_RADIUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::CYLINDER_HEIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::CYLINDER_RADIUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::CONE_HEIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::CONE_RADIUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SolidPrimitive_<ContainerAllocator>::PRISM_HEIGHT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace shape_msgs

#endif  // DIMOS_CDR_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Bool.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/bool.hpp"


#ifndef DIMOS_CDR_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B
#define DIMOS_CDR_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Bool __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Bool __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Bool_
{
  using Type = Bool_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Bool";

  explicit Bool_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = false;
    }
  }

  explicit Bool_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = false;
    }
  }

  // field types and members
  using _data_type =
    bool;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const bool & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Bool_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Bool_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Bool_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Bool_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Bool_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Bool_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Bool_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Bool_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Bool_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Bool_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Bool
    std::shared_ptr<std_msgs::msg::Bool_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Bool
    std::shared_ptr<std_msgs::msg::Bool_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bool_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bool_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bool_

// alias to use template instance with default allocator
using Bool =
  std_msgs::msg::Bool_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Byte.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/byte.hpp"


#ifndef DIMOS_CDR_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141
#define DIMOS_CDR_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Byte __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Byte __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Byte_
{
  using Type = Byte_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Byte";

  explicit Byte_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  explicit Byte_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  // field types and members
  using _data_type =
    unsigned char;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const unsigned char & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Byte_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Byte_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Byte_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Byte_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Byte_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Byte_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Byte_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Byte_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Byte_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Byte_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Byte
    std::shared_ptr<std_msgs::msg::Byte_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Byte
    std::shared_ptr<std_msgs::msg::Byte_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Byte_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Byte_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Byte_

// alias to use template instance with default allocator
using Byte =
  std_msgs::msg::Byte_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/MultiArrayDimension.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/multi_array_dimension.hpp"


#ifndef DIMOS_CDR_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2
#define DIMOS_CDR_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__MultiArrayDimension __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__MultiArrayDimension __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiArrayDimension_
{
  using Type = MultiArrayDimension_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/MultiArrayDimension";

  explicit MultiArrayDimension_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->size = 0ul;
      this->stride = 0ul;
    }
  }

  explicit MultiArrayDimension_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : label(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->size = 0ul;
      this->stride = 0ul;
    }
  }

  // field types and members
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _label_type label;
  using _size_type =
    uint32_t;
  _size_type size;
  using _stride_type =
    uint32_t;
  _stride_type stride;

  // setters for named parameter idiom
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__size(
    const uint32_t & _arg)
  {
    this->size = _arg;
    return *this;
  }
  Type & set__stride(
    const uint32_t & _arg)
  {
    this->stride = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::MultiArrayDimension_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::MultiArrayDimension_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::MultiArrayDimension_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::MultiArrayDimension_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::MultiArrayDimension_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::MultiArrayDimension_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::MultiArrayDimension_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::MultiArrayDimension_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::MultiArrayDimension_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::MultiArrayDimension_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__MultiArrayDimension
    std::shared_ptr<std_msgs::msg::MultiArrayDimension_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__MultiArrayDimension
    std::shared_ptr<std_msgs::msg::MultiArrayDimension_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiArrayDimension_ & other) const
  {
    if (this->label != other.label) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    if (this->stride != other.stride) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiArrayDimension_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiArrayDimension_

// alias to use template instance with default allocator
using MultiArrayDimension =
  std_msgs::msg::MultiArrayDimension_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/MultiArrayLayout.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/multi_array_layout.hpp"


#ifndef DIMOS_CDR_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8
#define DIMOS_CDR_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'dim'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__MultiArrayLayout __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__MultiArrayLayout __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiArrayLayout_
{
  using Type = MultiArrayLayout_<ContainerAllocator>;
void validate() const {
for (const auto& item : dim) { item.validate(); }
}
static constexpr const char* msg_name = "std_msgs/msg/MultiArrayLayout";

  explicit MultiArrayLayout_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_offset = 0ul;
    }
  }

  explicit MultiArrayLayout_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_offset = 0ul;
    }
  }

  // field types and members
  using _dim_type =
    std::vector<std_msgs::msg::MultiArrayDimension_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::MultiArrayDimension_<ContainerAllocator>>>;
  _dim_type dim;
  using _data_offset_type =
    uint32_t;
  _data_offset_type data_offset;

  // setters for named parameter idiom
  Type & set__dim(
    const std::vector<std_msgs::msg::MultiArrayDimension_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::MultiArrayDimension_<ContainerAllocator>>> & _arg)
  {
    this->dim = _arg;
    return *this;
  }
  Type & set__data_offset(
    const uint32_t & _arg)
  {
    this->data_offset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::MultiArrayLayout_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::MultiArrayLayout_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::MultiArrayLayout_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::MultiArrayLayout_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::MultiArrayLayout_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::MultiArrayLayout_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::MultiArrayLayout_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::MultiArrayLayout_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__MultiArrayLayout
    std::shared_ptr<std_msgs::msg::MultiArrayLayout_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__MultiArrayLayout
    std::shared_ptr<std_msgs::msg::MultiArrayLayout_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiArrayLayout_ & other) const
  {
    if (this->dim != other.dim) {
      return false;
    }
    if (this->data_offset != other.data_offset) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiArrayLayout_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiArrayLayout_

// alias to use template instance with default allocator
using MultiArrayLayout =
  std_msgs::msg::MultiArrayLayout_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/ByteMultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/byte_multi_array.hpp"


#ifndef DIMOS_CDR_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39
#define DIMOS_CDR_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__ByteMultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__ByteMultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ByteMultiArray_
{
  using Type = ByteMultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/ByteMultiArray";

  explicit ByteMultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit ByteMultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::ByteMultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::ByteMultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::ByteMultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::ByteMultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::ByteMultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::ByteMultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::ByteMultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::ByteMultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::ByteMultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::ByteMultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__ByteMultiArray
    std::shared_ptr<std_msgs::msg::ByteMultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__ByteMultiArray
    std::shared_ptr<std_msgs::msg::ByteMultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ByteMultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const ByteMultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ByteMultiArray_

// alias to use template instance with default allocator
using ByteMultiArray =
  std_msgs::msg::ByteMultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Char.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/char.hpp"


#ifndef DIMOS_CDR_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7
#define DIMOS_CDR_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Char __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Char __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Char_
{
  using Type = Char_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Char";

  explicit Char_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  explicit Char_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  // field types and members
  using _data_type =
    uint8_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const uint8_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Char_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Char_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Char_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Char_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Char_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Char_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Char_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Char_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Char_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Char_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Char
    std::shared_ptr<std_msgs::msg::Char_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Char
    std::shared_ptr<std_msgs::msg::Char_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Char_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Char_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Char_

// alias to use template instance with default allocator
using Char =
  std_msgs::msg::Char_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/ColorRGBA.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/color_rgba.hpp"


#ifndef DIMOS_CDR_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F
#define DIMOS_CDR_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__ColorRGBA __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__ColorRGBA __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ColorRGBA_
{
  using Type = ColorRGBA_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/ColorRGBA";

  explicit ColorRGBA_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0.0f;
      this->g = 0.0f;
      this->b = 0.0f;
      this->a = 0.0f;
    }
  }

  explicit ColorRGBA_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0.0f;
      this->g = 0.0f;
      this->b = 0.0f;
      this->a = 0.0f;
    }
  }

  // field types and members
  using _r_type =
    float;
  _r_type r;
  using _g_type =
    float;
  _g_type g;
  using _b_type =
    float;
  _b_type b;
  using _a_type =
    float;
  _a_type a;

  // setters for named parameter idiom
  Type & set__r(
    const float & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__g(
    const float & _arg)
  {
    this->g = _arg;
    return *this;
  }
  Type & set__b(
    const float & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__a(
    const float & _arg)
  {
    this->a = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::ColorRGBA_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::ColorRGBA_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::ColorRGBA_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::ColorRGBA_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::ColorRGBA_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::ColorRGBA_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::ColorRGBA_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::ColorRGBA_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::ColorRGBA_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__ColorRGBA
    std::shared_ptr<std_msgs::msg::ColorRGBA_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__ColorRGBA
    std::shared_ptr<std_msgs::msg::ColorRGBA_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ColorRGBA_ & other) const
  {
    if (this->r != other.r) {
      return false;
    }
    if (this->g != other.g) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    return true;
  }
  bool operator!=(const ColorRGBA_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ColorRGBA_

// alias to use template instance with default allocator
using ColorRGBA =
  std_msgs::msg::ColorRGBA_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Empty.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/empty.hpp"


#ifndef DIMOS_CDR_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380
#define DIMOS_CDR_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Empty __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Empty __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Empty_
{
  using Type = Empty_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Empty";

  explicit Empty_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Empty_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Empty_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Empty_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Empty_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Empty_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Empty_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Empty_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Empty_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Empty_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Empty_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Empty_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Empty
    std::shared_ptr<std_msgs::msg::Empty_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Empty
    std::shared_ptr<std_msgs::msg::Empty_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Empty_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Empty_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Empty_

// alias to use template instance with default allocator
using Empty =
  std_msgs::msg::Empty_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Float32.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/float32.hpp"


#ifndef DIMOS_CDR_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC
#define DIMOS_CDR_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Float32 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Float32 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Float32_
{
  using Type = Float32_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Float32";

  explicit Float32_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0.0f;
    }
  }

  explicit Float32_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0.0f;
    }
  }

  // field types and members
  using _data_type =
    float;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const float & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Float32_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Float32_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Float32_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Float32_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Float32_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Float32_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Float32_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Float32_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Float32_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Float32_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Float32
    std::shared_ptr<std_msgs::msg::Float32_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Float32
    std::shared_ptr<std_msgs::msg::Float32_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Float32_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Float32_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Float32_

// alias to use template instance with default allocator
using Float32 =
  std_msgs::msg::Float32_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Float32MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/float32_multi_array.hpp"


#ifndef DIMOS_CDR_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1
#define DIMOS_CDR_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Float32MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Float32MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Float32MultiArray_
{
  using Type = Float32MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Float32MultiArray";

  explicit Float32MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit Float32MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Float32MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Float32MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Float32MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Float32MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Float32MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Float32MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Float32MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Float32MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Float32MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Float32MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Float32MultiArray
    std::shared_ptr<std_msgs::msg::Float32MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Float32MultiArray
    std::shared_ptr<std_msgs::msg::Float32MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Float32MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Float32MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Float32MultiArray_

// alias to use template instance with default allocator
using Float32MultiArray =
  std_msgs::msg::Float32MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Float64.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/float64.hpp"


#ifndef DIMOS_CDR_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33
#define DIMOS_CDR_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Float64 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Float64 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Float64_
{
  using Type = Float64_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Float64";

  explicit Float64_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0.0;
    }
  }

  explicit Float64_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0.0;
    }
  }

  // field types and members
  using _data_type =
    double;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const double & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Float64_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Float64_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Float64_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Float64_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Float64_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Float64_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Float64_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Float64_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Float64_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Float64_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Float64
    std::shared_ptr<std_msgs::msg::Float64_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Float64
    std::shared_ptr<std_msgs::msg::Float64_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Float64_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Float64_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Float64_

// alias to use template instance with default allocator
using Float64 =
  std_msgs::msg::Float64_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Float64MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/float64_multi_array.hpp"


#ifndef DIMOS_CDR_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330
#define DIMOS_CDR_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Float64MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Float64MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Float64MultiArray_
{
  using Type = Float64MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Float64MultiArray";

  explicit Float64MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit Float64MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Float64MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Float64MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Float64MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Float64MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Float64MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Float64MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Float64MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Float64MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Float64MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Float64MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Float64MultiArray
    std::shared_ptr<std_msgs::msg::Float64MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Float64MultiArray
    std::shared_ptr<std_msgs::msg::Float64MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Float64MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Float64MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Float64MultiArray_

// alias to use template instance with default allocator
using Float64MultiArray =
  std_msgs::msg::Float64MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Int16.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/int16.hpp"


#ifndef DIMOS_CDR_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731
#define DIMOS_CDR_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Int16 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Int16 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int16_
{
  using Type = Int16_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Int16";

  explicit Int16_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  explicit Int16_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  // field types and members
  using _data_type =
    int16_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const int16_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Int16_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Int16_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Int16_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Int16_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int16_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int16_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int16_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int16_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Int16_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Int16_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Int16
    std::shared_ptr<std_msgs::msg::Int16_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Int16
    std::shared_ptr<std_msgs::msg::Int16_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int16_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int16_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int16_

// alias to use template instance with default allocator
using Int16 =
  std_msgs::msg::Int16_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Int16MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/int16_multi_array.hpp"


#ifndef DIMOS_CDR_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119
#define DIMOS_CDR_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Int16MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Int16MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int16MultiArray_
{
  using Type = Int16MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Int16MultiArray";

  explicit Int16MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit Int16MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<int16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int16_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Int16MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Int16MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Int16MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Int16MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int16MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int16MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int16MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int16MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Int16MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Int16MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Int16MultiArray
    std::shared_ptr<std_msgs::msg::Int16MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Int16MultiArray
    std::shared_ptr<std_msgs::msg::Int16MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int16MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int16MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int16MultiArray_

// alias to use template instance with default allocator
using Int16MultiArray =
  std_msgs::msg::Int16MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Int32.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/int32.hpp"


#ifndef DIMOS_CDR_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488
#define DIMOS_CDR_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Int32 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Int32 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int32_
{
  using Type = Int32_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Int32";

  explicit Int32_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0l;
    }
  }

  explicit Int32_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0l;
    }
  }

  // field types and members
  using _data_type =
    int32_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const int32_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Int32_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Int32_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Int32_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Int32_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int32_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int32_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int32_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int32_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Int32_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Int32_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Int32
    std::shared_ptr<std_msgs::msg::Int32_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Int32
    std::shared_ptr<std_msgs::msg::Int32_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int32_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int32_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int32_

// alias to use template instance with default allocator
using Int32 =
  std_msgs::msg::Int32_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Int32MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/int32_multi_array.hpp"


#ifndef DIMOS_CDR_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969
#define DIMOS_CDR_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Int32MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Int32MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int32MultiArray_
{
  using Type = Int32MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Int32MultiArray";

  explicit Int32MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit Int32MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Int32MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Int32MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Int32MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Int32MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int32MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int32MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int32MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int32MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Int32MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Int32MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Int32MultiArray
    std::shared_ptr<std_msgs::msg::Int32MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Int32MultiArray
    std::shared_ptr<std_msgs::msg::Int32MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int32MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int32MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int32MultiArray_

// alias to use template instance with default allocator
using Int32MultiArray =
  std_msgs::msg::Int32MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Int64.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/int64.hpp"


#ifndef DIMOS_CDR_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D
#define DIMOS_CDR_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Int64 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Int64 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int64_
{
  using Type = Int64_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Int64";

  explicit Int64_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0ll;
    }
  }

  explicit Int64_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0ll;
    }
  }

  // field types and members
  using _data_type =
    int64_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const int64_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Int64_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Int64_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Int64_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Int64_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int64_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int64_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int64_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int64_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Int64_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Int64_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Int64
    std::shared_ptr<std_msgs::msg::Int64_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Int64
    std::shared_ptr<std_msgs::msg::Int64_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int64_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int64_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int64_

// alias to use template instance with default allocator
using Int64 =
  std_msgs::msg::Int64_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Int64MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/int64_multi_array.hpp"


#ifndef DIMOS_CDR_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F
#define DIMOS_CDR_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Int64MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Int64MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int64MultiArray_
{
  using Type = Int64MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Int64MultiArray";

  explicit Int64MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit Int64MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Int64MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Int64MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Int64MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Int64MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int64MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int64MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int64MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int64MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Int64MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Int64MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Int64MultiArray
    std::shared_ptr<std_msgs::msg::Int64MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Int64MultiArray
    std::shared_ptr<std_msgs::msg::Int64MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int64MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int64MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int64MultiArray_

// alias to use template instance with default allocator
using Int64MultiArray =
  std_msgs::msg::Int64MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Int8.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/int8.hpp"


#ifndef DIMOS_CDR_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575
#define DIMOS_CDR_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Int8 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Int8 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int8_
{
  using Type = Int8_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/Int8";

  explicit Int8_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  explicit Int8_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  // field types and members
  using _data_type =
    int8_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const int8_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Int8_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Int8_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Int8_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Int8_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int8_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int8_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int8_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int8_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Int8_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Int8_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Int8
    std::shared_ptr<std_msgs::msg::Int8_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Int8
    std::shared_ptr<std_msgs::msg::Int8_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int8_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int8_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int8_

// alias to use template instance with default allocator
using Int8 =
  std_msgs::msg::Int8_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/Int8MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/int8_multi_array.hpp"


#ifndef DIMOS_CDR_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C
#define DIMOS_CDR_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__Int8MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__Int8MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Int8MultiArray_
{
  using Type = Int8MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/Int8MultiArray";

  explicit Int8MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit Int8MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::Int8MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::Int8MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::Int8MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::Int8MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int8MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int8MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::Int8MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::Int8MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::Int8MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::Int8MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__Int8MultiArray
    std::shared_ptr<std_msgs::msg::Int8MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__Int8MultiArray
    std::shared_ptr<std_msgs::msg::Int8MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Int8MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Int8MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Int8MultiArray_

// alias to use template instance with default allocator
using Int8MultiArray =
  std_msgs::msg::Int8MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/String.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/string.hpp"


#ifndef DIMOS_CDR_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1
#define DIMOS_CDR_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__String __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__String __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct String_
{
  using Type = String_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/String";

  explicit String_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  explicit String_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::String_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::String_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::String_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::String_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::String_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::String_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::String_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::String_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::String_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::String_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__String
    std::shared_ptr<std_msgs::msg::String_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__String
    std::shared_ptr<std_msgs::msg::String_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const String_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const String_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct String_

// alias to use template instance with default allocator
using String =
  std_msgs::msg::String_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/UInt16.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/u_int16.hpp"


#ifndef DIMOS_CDR_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5
#define DIMOS_CDR_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__UInt16 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__UInt16 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt16_
{
  using Type = UInt16_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/UInt16";

  explicit UInt16_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  explicit UInt16_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  // field types and members
  using _data_type =
    uint16_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const uint16_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::UInt16_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::UInt16_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::UInt16_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::UInt16_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt16_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt16_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt16_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt16_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::UInt16_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::UInt16_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__UInt16
    std::shared_ptr<std_msgs::msg::UInt16_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__UInt16
    std::shared_ptr<std_msgs::msg::UInt16_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt16_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt16_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt16_

// alias to use template instance with default allocator
using UInt16 =
  std_msgs::msg::UInt16_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/UInt16MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/u_int16_multi_array.hpp"


#ifndef DIMOS_CDR_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81
#define DIMOS_CDR_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__UInt16MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__UInt16MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt16MultiArray_
{
  using Type = UInt16MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/UInt16MultiArray";

  explicit UInt16MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit UInt16MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::UInt16MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::UInt16MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::UInt16MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::UInt16MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt16MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt16MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt16MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt16MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::UInt16MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::UInt16MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__UInt16MultiArray
    std::shared_ptr<std_msgs::msg::UInt16MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__UInt16MultiArray
    std::shared_ptr<std_msgs::msg::UInt16MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt16MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt16MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt16MultiArray_

// alias to use template instance with default allocator
using UInt16MultiArray =
  std_msgs::msg::UInt16MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/UInt32.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/u_int32.hpp"


#ifndef DIMOS_CDR_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F
#define DIMOS_CDR_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__UInt32 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__UInt32 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt32_
{
  using Type = UInt32_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/UInt32";

  explicit UInt32_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0ul;
    }
  }

  explicit UInt32_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0ul;
    }
  }

  // field types and members
  using _data_type =
    uint32_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const uint32_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::UInt32_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::UInt32_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::UInt32_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::UInt32_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt32_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt32_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt32_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt32_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::UInt32_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::UInt32_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__UInt32
    std::shared_ptr<std_msgs::msg::UInt32_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__UInt32
    std::shared_ptr<std_msgs::msg::UInt32_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt32_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt32_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt32_

// alias to use template instance with default allocator
using UInt32 =
  std_msgs::msg::UInt32_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/UInt32MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/u_int32_multi_array.hpp"


#ifndef DIMOS_CDR_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD
#define DIMOS_CDR_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__UInt32MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__UInt32MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt32MultiArray_
{
  using Type = UInt32MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/UInt32MultiArray";

  explicit UInt32MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit UInt32MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::UInt32MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::UInt32MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::UInt32MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::UInt32MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt32MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt32MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt32MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt32MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::UInt32MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::UInt32MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__UInt32MultiArray
    std::shared_ptr<std_msgs::msg::UInt32MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__UInt32MultiArray
    std::shared_ptr<std_msgs::msg::UInt32MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt32MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt32MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt32MultiArray_

// alias to use template instance with default allocator
using UInt32MultiArray =
  std_msgs::msg::UInt32MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/UInt64.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/u_int64.hpp"


#ifndef DIMOS_CDR_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7
#define DIMOS_CDR_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__UInt64 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__UInt64 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt64_
{
  using Type = UInt64_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/UInt64";

  explicit UInt64_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0ull;
    }
  }

  explicit UInt64_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0ull;
    }
  }

  // field types and members
  using _data_type =
    uint64_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const uint64_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::UInt64_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::UInt64_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::UInt64_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::UInt64_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt64_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt64_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt64_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt64_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::UInt64_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::UInt64_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__UInt64
    std::shared_ptr<std_msgs::msg::UInt64_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__UInt64
    std::shared_ptr<std_msgs::msg::UInt64_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt64_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt64_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt64_

// alias to use template instance with default allocator
using UInt64 =
  std_msgs::msg::UInt64_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/UInt64MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/u_int64_multi_array.hpp"


#ifndef DIMOS_CDR_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95
#define DIMOS_CDR_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__UInt64MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__UInt64MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt64MultiArray_
{
  using Type = UInt64MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/UInt64MultiArray";

  explicit UInt64MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit UInt64MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<uint64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint64_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint64_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::UInt64MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::UInt64MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::UInt64MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::UInt64MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt64MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt64MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt64MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt64MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::UInt64MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::UInt64MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__UInt64MultiArray
    std::shared_ptr<std_msgs::msg::UInt64MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__UInt64MultiArray
    std::shared_ptr<std_msgs::msg::UInt64MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt64MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt64MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt64MultiArray_

// alias to use template instance with default allocator
using UInt64MultiArray =
  std_msgs::msg::UInt64MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/UInt8.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/u_int8.hpp"


#ifndef DIMOS_CDR_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4
#define DIMOS_CDR_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__UInt8 __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__UInt8 __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt8_
{
  using Type = UInt8_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "std_msgs/msg/UInt8";

  explicit UInt8_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  explicit UInt8_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0;
    }
  }

  // field types and members
  using _data_type =
    uint8_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const uint8_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::UInt8_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::UInt8_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::UInt8_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::UInt8_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt8_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt8_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt8_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt8_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::UInt8_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::UInt8_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__UInt8
    std::shared_ptr<std_msgs::msg::UInt8_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__UInt8
    std::shared_ptr<std_msgs::msg::UInt8_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt8_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt8_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt8_

// alias to use template instance with default allocator
using UInt8 =
  std_msgs::msg::UInt8_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from std_msgs:msg/UInt8MultiArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "std_msgs/msg/u_int8_multi_array.hpp"


#ifndef DIMOS_CDR_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636
#define DIMOS_CDR_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'layout'

#ifndef _WIN32
# define DEPRECATED__std_msgs__msg__UInt8MultiArray __attribute__((deprecated))
#else
# define DEPRECATED__std_msgs__msg__UInt8MultiArray __declspec(deprecated)
#endif

namespace std_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UInt8MultiArray_
{
  using Type = UInt8MultiArray_<ContainerAllocator>;
void validate() const {
layout.validate();
}
static constexpr const char* msg_name = "std_msgs/msg/UInt8MultiArray";

  explicit UInt8MultiArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_init)
  {
    (void)_init;
  }

  explicit UInt8MultiArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : layout(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _layout_type =
    std_msgs::msg::MultiArrayLayout_<ContainerAllocator>;
  _layout_type layout;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__layout(
    const std_msgs::msg::MultiArrayLayout_<ContainerAllocator> & _arg)
  {
    this->layout = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    std_msgs::msg::UInt8MultiArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const std_msgs::msg::UInt8MultiArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<std_msgs::msg::UInt8MultiArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<std_msgs::msg::UInt8MultiArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt8MultiArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt8MultiArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      std_msgs::msg::UInt8MultiArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<std_msgs::msg::UInt8MultiArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<std_msgs::msg::UInt8MultiArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<std_msgs::msg::UInt8MultiArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__std_msgs__msg__UInt8MultiArray
    std::shared_ptr<std_msgs::msg::UInt8MultiArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__std_msgs__msg__UInt8MultiArray
    std::shared_ptr<std_msgs::msg::UInt8MultiArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UInt8MultiArray_ & other) const
  {
    if (this->layout != other.layout) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UInt8MultiArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UInt8MultiArray_

// alias to use template instance with default allocator
using UInt8MultiArray =
  std_msgs::msg::UInt8MultiArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace std_msgs

#endif  // DIMOS_CDR_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tf2_msgs:msg/TF2Error.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tf2_msgs/msg/tf2_error.hpp"


#ifndef DIMOS_CDR_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C
#define DIMOS_CDR_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__tf2_msgs__msg__TF2Error __attribute__((deprecated))
#else
# define DEPRECATED__tf2_msgs__msg__TF2Error __declspec(deprecated)
#endif

namespace tf2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TF2Error_
{
  using Type = TF2Error_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "tf2_msgs/msg/TF2Error";

  explicit TF2Error_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error = 0;
      this->error_string = "";
    }
  }

  explicit TF2Error_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : error_string(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error = 0;
      this->error_string = "";
    }
  }

  // field types and members
  using _error_type =
    uint8_t;
  _error_type error;
  using _error_string_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _error_string_type error_string;

  // setters for named parameter idiom
  Type & set__error(
    const uint8_t & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__error_string(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->error_string = _arg;
    return *this;
  }

  // constant declarations
  // guard against 'NO_ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(NO_ERROR)
#    pragma push_macro("NO_ERROR")
#    undef NO_ERROR
#  endif
#endif
  static constexpr uint8_t NO_ERROR =
    0u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("NO_ERROR")
#endif
  static constexpr uint8_t LOOKUP_ERROR =
    1u;
  static constexpr uint8_t CONNECTIVITY_ERROR =
    2u;
  static constexpr uint8_t EXTRAPOLATION_ERROR =
    3u;
  static constexpr uint8_t INVALID_ARGUMENT_ERROR =
    4u;
  static constexpr uint8_t TIMEOUT_ERROR =
    5u;
  static constexpr uint8_t TRANSFORM_ERROR =
    6u;

  // pointer types
  using RawPtr =
    tf2_msgs::msg::TF2Error_<ContainerAllocator> *;
  using ConstRawPtr =
    const tf2_msgs::msg::TF2Error_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tf2_msgs::msg::TF2Error_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tf2_msgs::msg::TF2Error_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tf2_msgs::msg::TF2Error_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tf2_msgs::msg::TF2Error_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tf2_msgs::msg::TF2Error_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tf2_msgs::msg::TF2Error_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tf2_msgs::msg::TF2Error_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tf2_msgs::msg::TF2Error_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tf2_msgs__msg__TF2Error
    std::shared_ptr<tf2_msgs::msg::TF2Error_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tf2_msgs__msg__TF2Error
    std::shared_ptr<tf2_msgs::msg::TF2Error_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TF2Error_ & other) const
  {
    if (this->error != other.error) {
      return false;
    }
    if (this->error_string != other.error_string) {
      return false;
    }
    return true;
  }
  bool operator!=(const TF2Error_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TF2Error_

// alias to use template instance with default allocator
using TF2Error =
  tf2_msgs::msg::TF2Error_<std::allocator<void>>;

// constant definitions
// guard against 'NO_ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(NO_ERROR)
#    pragma push_macro("NO_ERROR")
#    undef NO_ERROR
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TF2Error_<ContainerAllocator>::NO_ERROR;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("NO_ERROR")
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TF2Error_<ContainerAllocator>::LOOKUP_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TF2Error_<ContainerAllocator>::CONNECTIVITY_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TF2Error_<ContainerAllocator>::EXTRAPOLATION_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TF2Error_<ContainerAllocator>::INVALID_ARGUMENT_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TF2Error_<ContainerAllocator>::TIMEOUT_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t TF2Error_<ContainerAllocator>::TRANSFORM_ERROR;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace tf2_msgs

#endif  // DIMOS_CDR_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tf2_msgs:msg/TFMessage.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tf2_msgs/msg/tf_message.hpp"


#ifndef DIMOS_CDR_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A
#define DIMOS_CDR_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'transforms'

#ifndef _WIN32
# define DEPRECATED__tf2_msgs__msg__TFMessage __attribute__((deprecated))
#else
# define DEPRECATED__tf2_msgs__msg__TFMessage __declspec(deprecated)
#endif

namespace tf2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TFMessage_
{
  using Type = TFMessage_<ContainerAllocator>;
void validate() const {
for (const auto& item : transforms) { item.validate(); }
}
static constexpr const char* msg_name = "tf2_msgs/msg/TFMessage";

  explicit TFMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TFMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _transforms_type =
    std::vector<geometry_msgs::msg::TransformStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::TransformStamped_<ContainerAllocator>>>;
  _transforms_type transforms;

  // setters for named parameter idiom
  Type & set__transforms(
    const std::vector<geometry_msgs::msg::TransformStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::TransformStamped_<ContainerAllocator>>> & _arg)
  {
    this->transforms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tf2_msgs::msg::TFMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const tf2_msgs::msg::TFMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tf2_msgs::msg::TFMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tf2_msgs::msg::TFMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tf2_msgs::msg::TFMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tf2_msgs::msg::TFMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tf2_msgs::msg::TFMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tf2_msgs::msg::TFMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tf2_msgs::msg::TFMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tf2_msgs::msg::TFMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tf2_msgs__msg__TFMessage
    std::shared_ptr<tf2_msgs::msg::TFMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tf2_msgs__msg__TFMessage
    std::shared_ptr<tf2_msgs::msg::TFMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TFMessage_ & other) const
  {
    if (this->transforms != other.transforms) {
      return false;
    }
    return true;
  }
  bool operator!=(const TFMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TFMessage_

// alias to use template instance with default allocator
using TFMessage =
  tf2_msgs::msg::TFMessage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tf2_msgs

#endif  // DIMOS_CDR_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from trajectory_msgs:msg/JointTrajectoryPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "trajectory_msgs/msg/joint_trajectory_point.hpp"


#ifndef DIMOS_CDR_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848
#define DIMOS_CDR_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'time_from_start'

#ifndef _WIN32
# define DEPRECATED__trajectory_msgs__msg__JointTrajectoryPoint __attribute__((deprecated))
#else
# define DEPRECATED__trajectory_msgs__msg__JointTrajectoryPoint __declspec(deprecated)
#endif

namespace trajectory_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointTrajectoryPoint_
{
  using Type = JointTrajectoryPoint_<ContainerAllocator>;
void validate() const {
time_from_start.validate();
}
static constexpr const char* msg_name = "trajectory_msgs/msg/JointTrajectoryPoint";

  explicit JointTrajectoryPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time_from_start(_init)
  {
    (void)_init;
  }

  explicit JointTrajectoryPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time_from_start(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _positions_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _positions_type positions;
  using _velocities_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _velocities_type velocities;
  using _accelerations_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _accelerations_type accelerations;
  using _effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _effort_type effort;
  using _time_from_start_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _time_from_start_type time_from_start;

  // setters for named parameter idiom
  Type & set__positions(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->positions = _arg;
    return *this;
  }
  Type & set__velocities(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->velocities = _arg;
    return *this;
  }
  Type & set__accelerations(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->accelerations = _arg;
    return *this;
  }
  Type & set__effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->effort = _arg;
    return *this;
  }
  Type & set__time_from_start(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->time_from_start = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__trajectory_msgs__msg__JointTrajectoryPoint
    std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__trajectory_msgs__msg__JointTrajectoryPoint
    std::shared_ptr<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointTrajectoryPoint_ & other) const
  {
    if (this->positions != other.positions) {
      return false;
    }
    if (this->velocities != other.velocities) {
      return false;
    }
    if (this->accelerations != other.accelerations) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    if (this->time_from_start != other.time_from_start) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointTrajectoryPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointTrajectoryPoint_

// alias to use template instance with default allocator
using JointTrajectoryPoint =
  trajectory_msgs::msg::JointTrajectoryPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace trajectory_msgs

#endif  // DIMOS_CDR_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from trajectory_msgs:msg/JointTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "trajectory_msgs/msg/joint_trajectory.hpp"


#ifndef DIMOS_CDR_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56
#define DIMOS_CDR_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'points'

#ifndef _WIN32
# define DEPRECATED__trajectory_msgs__msg__JointTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__trajectory_msgs__msg__JointTrajectory __declspec(deprecated)
#endif

namespace trajectory_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointTrajectory_
{
  using Type = JointTrajectory_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : points) { item.validate(); }
}
static constexpr const char* msg_name = "trajectory_msgs/msg/JointTrajectory";

  explicit JointTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit JointTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _joint_names_type joint_names;
  using _points_type =
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->joint_names = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<trajectory_msgs::msg::JointTrajectoryPoint_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    trajectory_msgs::msg::JointTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const trajectory_msgs::msg::JointTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      trajectory_msgs::msg::JointTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<trajectory_msgs::msg::JointTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      trajectory_msgs::msg::JointTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<trajectory_msgs::msg::JointTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<trajectory_msgs::msg::JointTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<trajectory_msgs::msg::JointTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__trajectory_msgs__msg__JointTrajectory
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__trajectory_msgs__msg__JointTrajectory
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointTrajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_names != other.joint_names) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointTrajectory_

// alias to use template instance with default allocator
using JointTrajectory =
  trajectory_msgs::msg::JointTrajectory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace trajectory_msgs

#endif  // DIMOS_CDR_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from trajectory_msgs:msg/MultiDOFJointTrajectoryPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"


#ifndef DIMOS_CDR_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA
#define DIMOS_CDR_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'transforms'
// Member 'velocities'
// Member 'accelerations'
// Member 'time_from_start'

#ifndef _WIN32
# define DEPRECATED__trajectory_msgs__msg__MultiDOFJointTrajectoryPoint __attribute__((deprecated))
#else
# define DEPRECATED__trajectory_msgs__msg__MultiDOFJointTrajectoryPoint __declspec(deprecated)
#endif

namespace trajectory_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiDOFJointTrajectoryPoint_
{
  using Type = MultiDOFJointTrajectoryPoint_<ContainerAllocator>;
void validate() const {
for (const auto& item : transforms) { item.validate(); }
for (const auto& item : velocities) { item.validate(); }
for (const auto& item : accelerations) { item.validate(); }
time_from_start.validate();
}
static constexpr const char* msg_name = "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint";

  explicit MultiDOFJointTrajectoryPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time_from_start(_init)
  {
    (void)_init;
  }

  explicit MultiDOFJointTrajectoryPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time_from_start(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _transforms_type =
    std::vector<geometry_msgs::msg::Transform_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Transform_<ContainerAllocator>>>;
  _transforms_type transforms;
  using _velocities_type =
    std::vector<geometry_msgs::msg::Twist_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Twist_<ContainerAllocator>>>;
  _velocities_type velocities;
  using _accelerations_type =
    std::vector<geometry_msgs::msg::Twist_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Twist_<ContainerAllocator>>>;
  _accelerations_type accelerations;
  using _time_from_start_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _time_from_start_type time_from_start;

  // setters for named parameter idiom
  Type & set__transforms(
    const std::vector<geometry_msgs::msg::Transform_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Transform_<ContainerAllocator>>> & _arg)
  {
    this->transforms = _arg;
    return *this;
  }
  Type & set__velocities(
    const std::vector<geometry_msgs::msg::Twist_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Twist_<ContainerAllocator>>> & _arg)
  {
    this->velocities = _arg;
    return *this;
  }
  Type & set__accelerations(
    const std::vector<geometry_msgs::msg::Twist_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Twist_<ContainerAllocator>>> & _arg)
  {
    this->accelerations = _arg;
    return *this;
  }
  Type & set__time_from_start(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->time_from_start = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__trajectory_msgs__msg__MultiDOFJointTrajectoryPoint
    std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__trajectory_msgs__msg__MultiDOFJointTrajectoryPoint
    std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiDOFJointTrajectoryPoint_ & other) const
  {
    if (this->transforms != other.transforms) {
      return false;
    }
    if (this->velocities != other.velocities) {
      return false;
    }
    if (this->accelerations != other.accelerations) {
      return false;
    }
    if (this->time_from_start != other.time_from_start) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiDOFJointTrajectoryPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiDOFJointTrajectoryPoint_

// alias to use template instance with default allocator
using MultiDOFJointTrajectoryPoint =
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace trajectory_msgs

#endif  // DIMOS_CDR_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from trajectory_msgs:msg/MultiDOFJointTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"


#ifndef DIMOS_CDR_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2
#define DIMOS_CDR_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'points'

#ifndef _WIN32
# define DEPRECATED__trajectory_msgs__msg__MultiDOFJointTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__trajectory_msgs__msg__MultiDOFJointTrajectory __declspec(deprecated)
#endif

namespace trajectory_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiDOFJointTrajectory_
{
  using Type = MultiDOFJointTrajectory_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : points) { item.validate(); }
}
static constexpr const char* msg_name = "trajectory_msgs/msg/MultiDOFJointTrajectory";

  explicit MultiDOFJointTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MultiDOFJointTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _joint_names_type joint_names;
  using _points_type =
    std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->joint_names = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__trajectory_msgs__msg__MultiDOFJointTrajectory
    std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__trajectory_msgs__msg__MultiDOFJointTrajectory
    std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiDOFJointTrajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_names != other.joint_names) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiDOFJointTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiDOFJointTrajectory_

// alias to use template instance with default allocator
using MultiDOFJointTrajectory =
  trajectory_msgs::msg::MultiDOFJointTrajectory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace trajectory_msgs

#endif  // DIMOS_CDR_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/BoundingBox2DArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/bounding_box2_d_array.hpp"


#ifndef DIMOS_CDR_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E
#define DIMOS_CDR_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'boxes'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__BoundingBox2DArray __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__BoundingBox2DArray __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox2DArray_
{
  using Type = BoundingBox2DArray_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : boxes) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/BoundingBox2DArray";

  explicit BoundingBox2DArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit BoundingBox2DArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _boxes_type =
    std::vector<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>>;
  _boxes_type boxes;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__boxes(
    const std::vector<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox2D_<ContainerAllocator>>> & _arg)
  {
    this->boxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__BoundingBox2DArray
    std::shared_ptr<vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__BoundingBox2DArray
    std::shared_ptr<vision_msgs::msg::BoundingBox2DArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox2DArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->boxes != other.boxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox2DArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox2DArray_

// alias to use template instance with default allocator
using BoundingBox2DArray =
  vision_msgs::msg::BoundingBox2DArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/BoundingBox3DArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/bounding_box3_d_array.hpp"


#ifndef DIMOS_CDR_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65
#define DIMOS_CDR_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'boxes'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__BoundingBox3DArray __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__BoundingBox3DArray __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox3DArray_
{
  using Type = BoundingBox3DArray_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : boxes) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/BoundingBox3DArray";

  explicit BoundingBox3DArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit BoundingBox3DArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _boxes_type =
    std::vector<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>>;
  _boxes_type boxes;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__boxes(
    const std::vector<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::BoundingBox3D_<ContainerAllocator>>> & _arg)
  {
    this->boxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__BoundingBox3DArray
    std::shared_ptr<vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__BoundingBox3DArray
    std::shared_ptr<vision_msgs::msg::BoundingBox3DArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox3DArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->boxes != other.boxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox3DArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox3DArray_

// alias to use template instance with default allocator
using BoundingBox3DArray =
  vision_msgs::msg::BoundingBox3DArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/ObjectHypothesis.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/object_hypothesis.hpp"


#ifndef DIMOS_CDR_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A
#define DIMOS_CDR_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__ObjectHypothesis __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__ObjectHypothesis __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectHypothesis_
{
  using Type = ObjectHypothesis_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "vision_msgs/msg/ObjectHypothesis";

  explicit ObjectHypothesis_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = "";
      this->score = 0.0;
    }
  }

  explicit ObjectHypothesis_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = "";
      this->score = 0.0;
    }
  }

  // field types and members
  using _class_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_id_type class_id;
  using _score_type =
    double;
  _score_type score;

  // setters for named parameter idiom
  Type & set__class_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__score(
    const double & _arg)
  {
    this->score = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::ObjectHypothesis_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::ObjectHypothesis_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__ObjectHypothesis
    std::shared_ptr<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__ObjectHypothesis
    std::shared_ptr<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectHypothesis_ & other) const
  {
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectHypothesis_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectHypothesis_

// alias to use template instance with default allocator
using ObjectHypothesis =
  vision_msgs::msg::ObjectHypothesis_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/Classification.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/classification.hpp"


#ifndef DIMOS_CDR_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339
#define DIMOS_CDR_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'results'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__Classification __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__Classification __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Classification_
{
  using Type = Classification_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : results) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/Classification";

  explicit Classification_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Classification_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _results_type =
    std::vector<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>>>;
  _results_type results;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__results(
    const std::vector<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>>> & _arg)
  {
    this->results = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::Classification_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::Classification_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::Classification_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::Classification_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Classification_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Classification_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Classification_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Classification_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::Classification_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::Classification_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__Classification
    std::shared_ptr<vision_msgs::msg::Classification_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__Classification
    std::shared_ptr<vision_msgs::msg::Classification_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Classification_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->results != other.results) {
      return false;
    }
    return true;
  }
  bool operator!=(const Classification_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Classification_

// alias to use template instance with default allocator
using Classification =
  vision_msgs::msg::Classification_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/ObjectHypothesisWithPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/object_hypothesis_with_pose.hpp"


#ifndef DIMOS_CDR_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1
#define DIMOS_CDR_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'hypothesis'
// Member 'pose'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__ObjectHypothesisWithPose __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__ObjectHypothesisWithPose __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectHypothesisWithPose_
{
  using Type = ObjectHypothesisWithPose_<ContainerAllocator>;
void validate() const {
hypothesis.validate();
pose.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/ObjectHypothesisWithPose";

  explicit ObjectHypothesisWithPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : hypothesis(_init),
    pose(_init)
  {
    (void)_init;
  }

  explicit ObjectHypothesisWithPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : hypothesis(_alloc, _init),
    pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _hypothesis_type =
    vision_msgs::msg::ObjectHypothesis_<ContainerAllocator>;
  _hypothesis_type hypothesis;
  using _pose_type =
    geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__hypothesis(
    const vision_msgs::msg::ObjectHypothesis_<ContainerAllocator> & _arg)
  {
    this->hypothesis = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::PoseWithCovariance_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__ObjectHypothesisWithPose
    std::shared_ptr<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__ObjectHypothesisWithPose
    std::shared_ptr<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectHypothesisWithPose_ & other) const
  {
    if (this->hypothesis != other.hypothesis) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectHypothesisWithPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectHypothesisWithPose_

// alias to use template instance with default allocator
using ObjectHypothesisWithPose =
  vision_msgs::msg::ObjectHypothesisWithPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/Detection2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/detection2_d.hpp"


#ifndef DIMOS_CDR_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29
#define DIMOS_CDR_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'results'
// Member 'bbox'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__Detection2D __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__Detection2D __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection2D_
{
  using Type = Detection2D_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : results) { item.validate(); }
bbox.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/Detection2D";

  explicit Detection2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    bbox(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
    }
  }

  explicit Detection2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    bbox(_alloc, _init),
    id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _results_type =
    std::vector<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>>;
  _results_type results;
  using _bbox_type =
    vision_msgs::msg::BoundingBox2D_<ContainerAllocator>;
  _bbox_type bbox;
  using _id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _id_type id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__results(
    const std::vector<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>> & _arg)
  {
    this->results = _arg;
    return *this;
  }
  Type & set__bbox(
    const vision_msgs::msg::BoundingBox2D_<ContainerAllocator> & _arg)
  {
    this->bbox = _arg;
    return *this;
  }
  Type & set__id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::Detection2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::Detection2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::Detection2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::Detection2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Detection2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Detection2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Detection2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Detection2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::Detection2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::Detection2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__Detection2D
    std::shared_ptr<vision_msgs::msg::Detection2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__Detection2D
    std::shared_ptr<vision_msgs::msg::Detection2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection2D_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->results != other.results) {
      return false;
    }
    if (this->bbox != other.bbox) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection2D_

// alias to use template instance with default allocator
using Detection2D =
  vision_msgs::msg::Detection2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/Detection2DArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/detection2_d_array.hpp"


#ifndef DIMOS_CDR_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A
#define DIMOS_CDR_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'detections'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__Detection2DArray __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__Detection2DArray __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection2DArray_
{
  using Type = Detection2DArray_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : detections) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/Detection2DArray";

  explicit Detection2DArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Detection2DArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detections_type =
    std::vector<vision_msgs::msg::Detection2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::Detection2D_<ContainerAllocator>>>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<vision_msgs::msg::Detection2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::Detection2D_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::Detection2DArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::Detection2DArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::Detection2DArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::Detection2DArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Detection2DArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Detection2DArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Detection2DArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Detection2DArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::Detection2DArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::Detection2DArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__Detection2DArray
    std::shared_ptr<vision_msgs::msg::Detection2DArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__Detection2DArray
    std::shared_ptr<vision_msgs::msg::Detection2DArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection2DArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection2DArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection2DArray_

// alias to use template instance with default allocator
using Detection2DArray =
  vision_msgs::msg::Detection2DArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/Detection3D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/detection3_d.hpp"


#ifndef DIMOS_CDR_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C
#define DIMOS_CDR_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'results'
// Member 'bbox'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__Detection3D __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__Detection3D __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection3D_
{
  using Type = Detection3D_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : results) { item.validate(); }
bbox.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/Detection3D";

  explicit Detection3D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    bbox(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
    }
  }

  explicit Detection3D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    bbox(_alloc, _init),
    id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _results_type =
    std::vector<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>>;
  _results_type results;
  using _bbox_type =
    vision_msgs::msg::BoundingBox3D_<ContainerAllocator>;
  _bbox_type bbox;
  using _id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _id_type id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__results(
    const std::vector<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::ObjectHypothesisWithPose_<ContainerAllocator>>> & _arg)
  {
    this->results = _arg;
    return *this;
  }
  Type & set__bbox(
    const vision_msgs::msg::BoundingBox3D_<ContainerAllocator> & _arg)
  {
    this->bbox = _arg;
    return *this;
  }
  Type & set__id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::Detection3D_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::Detection3D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::Detection3D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::Detection3D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Detection3D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Detection3D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Detection3D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Detection3D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::Detection3D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::Detection3D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__Detection3D
    std::shared_ptr<vision_msgs::msg::Detection3D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__Detection3D
    std::shared_ptr<vision_msgs::msg::Detection3D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection3D_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->results != other.results) {
      return false;
    }
    if (this->bbox != other.bbox) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection3D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection3D_

// alias to use template instance with default allocator
using Detection3D =
  vision_msgs::msg::Detection3D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/Detection3DArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/detection3_d_array.hpp"


#ifndef DIMOS_CDR_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE
#define DIMOS_CDR_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'detections'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__Detection3DArray __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__Detection3DArray __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection3DArray_
{
  using Type = Detection3DArray_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : detections) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/Detection3DArray";

  explicit Detection3DArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Detection3DArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detections_type =
    std::vector<vision_msgs::msg::Detection3D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::Detection3D_<ContainerAllocator>>>;
  _detections_type detections;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<vision_msgs::msg::Detection3D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::Detection3D_<ContainerAllocator>>> & _arg)
  {
    this->detections = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::Detection3DArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::Detection3DArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::Detection3DArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::Detection3DArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Detection3DArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Detection3DArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::Detection3DArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::Detection3DArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::Detection3DArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::Detection3DArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__Detection3DArray
    std::shared_ptr<vision_msgs::msg::Detection3DArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__Detection3DArray
    std::shared_ptr<vision_msgs::msg::Detection3DArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection3DArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection3DArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection3DArray_

// alias to use template instance with default allocator
using Detection3DArray =
  vision_msgs::msg::Detection3DArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/VisionClass.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/vision_class.hpp"


#ifndef DIMOS_CDR_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A
#define DIMOS_CDR_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__VisionClass __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__VisionClass __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VisionClass_
{
  using Type = VisionClass_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "vision_msgs/msg/VisionClass";

  explicit VisionClass_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0;
      this->class_name = "";
    }
  }

  explicit VisionClass_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0;
      this->class_name = "";
    }
  }

  // field types and members
  using _class_id_type =
    uint16_t;
  _class_id_type class_id;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;

  // setters for named parameter idiom
  Type & set__class_id(
    const uint16_t & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::VisionClass_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::VisionClass_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::VisionClass_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::VisionClass_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::VisionClass_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::VisionClass_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::VisionClass_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::VisionClass_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::VisionClass_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::VisionClass_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__VisionClass
    std::shared_ptr<vision_msgs::msg::VisionClass_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__VisionClass
    std::shared_ptr<vision_msgs::msg::VisionClass_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VisionClass_ & other) const
  {
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const VisionClass_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VisionClass_

// alias to use template instance with default allocator
using VisionClass =
  vision_msgs::msg::VisionClass_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/LabelInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/label_info.hpp"


#ifndef DIMOS_CDR_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062
#define DIMOS_CDR_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'class_map'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__LabelInfo __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__LabelInfo __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LabelInfo_
{
  using Type = LabelInfo_<ContainerAllocator>;
void validate() const {
header.validate();
for (const auto& item : class_map) { item.validate(); }
}
static constexpr const char* msg_name = "vision_msgs/msg/LabelInfo";

  explicit LabelInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->threshold = 0.0f;
    }
  }

  explicit LabelInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->threshold = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _class_map_type =
    std::vector<vision_msgs::msg::VisionClass_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::VisionClass_<ContainerAllocator>>>;
  _class_map_type class_map;
  using _threshold_type =
    float;
  _threshold_type threshold;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__class_map(
    const std::vector<vision_msgs::msg::VisionClass_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vision_msgs::msg::VisionClass_<ContainerAllocator>>> & _arg)
  {
    this->class_map = _arg;
    return *this;
  }
  Type & set__threshold(
    const float & _arg)
  {
    this->threshold = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::LabelInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::LabelInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::LabelInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::LabelInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::LabelInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::LabelInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::LabelInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::LabelInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::LabelInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::LabelInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__LabelInfo
    std::shared_ptr<vision_msgs::msg::LabelInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__LabelInfo
    std::shared_ptr<vision_msgs::msg::LabelInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LabelInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->class_map != other.class_map) {
      return false;
    }
    if (this->threshold != other.threshold) {
      return false;
    }
    return true;
  }
  bool operator!=(const LabelInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LabelInfo_

// alias to use template instance with default allocator
using LabelInfo =
  vision_msgs::msg::LabelInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_msgs:msg/VisionInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vision_msgs/msg/vision_info.hpp"


#ifndef DIMOS_CDR_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2
#define DIMOS_CDR_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'

#ifndef _WIN32
# define DEPRECATED__vision_msgs__msg__VisionInfo __attribute__((deprecated))
#else
# define DEPRECATED__vision_msgs__msg__VisionInfo __declspec(deprecated)
#endif

namespace vision_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VisionInfo_
{
  using Type = VisionInfo_<ContainerAllocator>;
void validate() const {
header.validate();
}
static constexpr const char* msg_name = "vision_msgs/msg/VisionInfo";

  explicit VisionInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->method = "";
      this->database_location = "";
      this->database_version = 0l;
    }
  }

  explicit VisionInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    method(_alloc),
    database_location(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->method = "";
      this->database_location = "";
      this->database_version = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _method_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _method_type method;
  using _database_location_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _database_location_type database_location;
  using _database_version_type =
    int32_t;
  _database_version_type database_version;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__method(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->method = _arg;
    return *this;
  }
  Type & set__database_location(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->database_location = _arg;
    return *this;
  }
  Type & set__database_version(
    const int32_t & _arg)
  {
    this->database_version = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_msgs::msg::VisionInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_msgs::msg::VisionInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_msgs::msg::VisionInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_msgs::msg::VisionInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::VisionInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::VisionInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_msgs::msg::VisionInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_msgs::msg::VisionInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_msgs::msg::VisionInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_msgs::msg::VisionInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_msgs__msg__VisionInfo
    std::shared_ptr<vision_msgs::msg::VisionInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_msgs__msg__VisionInfo
    std::shared_ptr<vision_msgs::msg::VisionInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VisionInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->method != other.method) {
      return false;
    }
    if (this->database_location != other.database_location) {
      return false;
    }
    if (this->database_version != other.database_version) {
      return false;
    }
    return true;
  }
  bool operator!=(const VisionInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VisionInfo_

// alias to use template instance with default allocator
using VisionInfo =
  vision_msgs::msg::VisionInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_msgs

#endif  // DIMOS_CDR_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/ImageMarker.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/image_marker.hpp"


#ifndef DIMOS_CDR_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC
#define DIMOS_CDR_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'position'
// Member 'points'
// Member 'outline_color'
// Member 'fill_color'
// Member 'outline_colors'
// Member 'lifetime'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__ImageMarker __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__ImageMarker __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImageMarker_
{
  using Type = ImageMarker_<ContainerAllocator>;
void validate() const {
header.validate();
position.validate();
outline_color.validate();
fill_color.validate();
lifetime.validate();
for (const auto& item : points) { item.validate(); }
for (const auto& item : outline_colors) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/ImageMarker";

  explicit ImageMarker_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    outline_color(_init),
    fill_color(_init),
    lifetime(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ns = "";
      this->id = 0l;
      this->type = 0l;
      this->action = 0l;
      this->scale = 0.0f;
      this->filled = 0;
    }
  }

  explicit ImageMarker_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    ns(_alloc),
    position(_alloc, _init),
    outline_color(_alloc, _init),
    fill_color(_alloc, _init),
    lifetime(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ns = "";
      this->id = 0l;
      this->type = 0l;
      this->action = 0l;
      this->scale = 0.0f;
      this->filled = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _ns_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ns_type ns;
  using _id_type =
    int32_t;
  _id_type id;
  using _type_type =
    int32_t;
  _type_type type;
  using _action_type =
    int32_t;
  _action_type action;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _scale_type =
    float;
  _scale_type scale;
  using _outline_color_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _outline_color_type outline_color;
  using _filled_type =
    uint8_t;
  _filled_type filled;
  using _fill_color_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _fill_color_type fill_color;
  using _lifetime_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _lifetime_type lifetime;
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;
  using _outline_colors_type =
    std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::ColorRGBA_<ContainerAllocator>>>;
  _outline_colors_type outline_colors;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__ns(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ns = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__type(
    const int32_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__action(
    const int32_t & _arg)
  {
    this->action = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__scale(
    const float & _arg)
  {
    this->scale = _arg;
    return *this;
  }
  Type & set__outline_color(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->outline_color = _arg;
    return *this;
  }
  Type & set__filled(
    const uint8_t & _arg)
  {
    this->filled = _arg;
    return *this;
  }
  Type & set__fill_color(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->fill_color = _arg;
    return *this;
  }
  Type & set__lifetime(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->lifetime = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__outline_colors(
    const std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::ColorRGBA_<ContainerAllocator>>> & _arg)
  {
    this->outline_colors = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t CIRCLE =
    0;
  static constexpr int32_t LINE_STRIP =
    1;
  static constexpr int32_t LINE_LIST =
    2;
  static constexpr int32_t POLYGON =
    3;
  static constexpr int32_t POINTS =
    4;
  static constexpr int32_t ADD =
    0;
  static constexpr int32_t REMOVE =
    1;

  // pointer types
  using RawPtr =
    visualization_msgs::msg::ImageMarker_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::ImageMarker_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::ImageMarker_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::ImageMarker_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::ImageMarker_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::ImageMarker_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::ImageMarker_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::ImageMarker_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::ImageMarker_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::ImageMarker_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__ImageMarker
    std::shared_ptr<visualization_msgs::msg::ImageMarker_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__ImageMarker
    std::shared_ptr<visualization_msgs::msg::ImageMarker_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImageMarker_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->ns != other.ns) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->action != other.action) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->scale != other.scale) {
      return false;
    }
    if (this->outline_color != other.outline_color) {
      return false;
    }
    if (this->filled != other.filled) {
      return false;
    }
    if (this->fill_color != other.fill_color) {
      return false;
    }
    if (this->lifetime != other.lifetime) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->outline_colors != other.outline_colors) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImageMarker_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImageMarker_

// alias to use template instance with default allocator
using ImageMarker =
  visualization_msgs::msg::ImageMarker_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ImageMarker_<ContainerAllocator>::CIRCLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ImageMarker_<ContainerAllocator>::LINE_STRIP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ImageMarker_<ContainerAllocator>::LINE_LIST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ImageMarker_<ContainerAllocator>::POLYGON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ImageMarker_<ContainerAllocator>::POINTS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ImageMarker_<ContainerAllocator>::ADD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ImageMarker_<ContainerAllocator>::REMOVE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/MeshFile.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/mesh_file.hpp"


#ifndef DIMOS_CDR_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264
#define DIMOS_CDR_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__MeshFile __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__MeshFile __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshFile_
{
  using Type = MeshFile_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "visualization_msgs/msg/MeshFile";

  explicit MeshFile_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->filename = "";
    }
  }

  explicit MeshFile_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : filename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->filename = "";
    }
  }

  // field types and members
  using _filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _filename_type filename;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->filename = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    visualization_msgs::msg::MeshFile_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::MeshFile_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::MeshFile_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::MeshFile_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::MeshFile_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::MeshFile_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::MeshFile_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::MeshFile_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::MeshFile_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::MeshFile_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__MeshFile
    std::shared_ptr<visualization_msgs::msg::MeshFile_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__MeshFile
    std::shared_ptr<visualization_msgs::msg::MeshFile_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshFile_ & other) const
  {
    if (this->filename != other.filename) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshFile_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshFile_

// alias to use template instance with default allocator
using MeshFile =
  visualization_msgs::msg::MeshFile_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/UVCoordinate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/uv_coordinate.hpp"


#ifndef DIMOS_CDR_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F
#define DIMOS_CDR_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__UVCoordinate __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__UVCoordinate __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UVCoordinate_
{
  using Type = UVCoordinate_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "visualization_msgs/msg/UVCoordinate";

  explicit UVCoordinate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u = 0.0f;
      this->v = 0.0f;
    }
  }

  explicit UVCoordinate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u = 0.0f;
      this->v = 0.0f;
    }
  }

  // field types and members
  using _u_type =
    float;
  _u_type u;
  using _v_type =
    float;
  _v_type v;

  // setters for named parameter idiom
  Type & set__u(
    const float & _arg)
  {
    this->u = _arg;
    return *this;
  }
  Type & set__v(
    const float & _arg)
  {
    this->v = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    visualization_msgs::msg::UVCoordinate_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::UVCoordinate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::UVCoordinate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::UVCoordinate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::UVCoordinate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::UVCoordinate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::UVCoordinate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::UVCoordinate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::UVCoordinate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::UVCoordinate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__UVCoordinate
    std::shared_ptr<visualization_msgs::msg::UVCoordinate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__UVCoordinate
    std::shared_ptr<visualization_msgs::msg::UVCoordinate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UVCoordinate_ & other) const
  {
    if (this->u != other.u) {
      return false;
    }
    if (this->v != other.v) {
      return false;
    }
    return true;
  }
  bool operator!=(const UVCoordinate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UVCoordinate_

// alias to use template instance with default allocator
using UVCoordinate =
  visualization_msgs::msg::UVCoordinate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/Marker.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/marker.hpp"


#ifndef DIMOS_CDR_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF
#define DIMOS_CDR_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'pose'
// Member 'scale'
// Member 'color'
// Member 'colors'
// Member 'lifetime'
// Member 'points'
// Member 'texture'
// Member 'uv_coordinates'
// Member 'mesh_file'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__Marker __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__Marker __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Marker_
{
  using Type = Marker_<ContainerAllocator>;
void validate() const {
header.validate();
pose.validate();
scale.validate();
color.validate();
lifetime.validate();
for (const auto& item : points) { item.validate(); }
for (const auto& item : colors) { item.validate(); }
texture.validate();
for (const auto& item : uv_coordinates) { item.validate(); }
mesh_file.validate();
}
static constexpr const char* msg_name = "visualization_msgs/msg/Marker";

  explicit Marker_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    scale(_init),
    color(_init),
    lifetime(_init),
    texture(_init),
    mesh_file(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ns = "";
      this->id = 0l;
      this->type = 0l;
      this->action = 0l;
      this->frame_locked = false;
      this->texture_resource = "";
      this->text = "";
      this->mesh_resource = "";
      this->mesh_use_embedded_materials = false;
    }
  }

  explicit Marker_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    ns(_alloc),
    pose(_alloc, _init),
    scale(_alloc, _init),
    color(_alloc, _init),
    lifetime(_alloc, _init),
    texture_resource(_alloc),
    texture(_alloc, _init),
    text(_alloc),
    mesh_resource(_alloc),
    mesh_file(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ns = "";
      this->id = 0l;
      this->type = 0l;
      this->action = 0l;
      this->frame_locked = false;
      this->texture_resource = "";
      this->text = "";
      this->mesh_resource = "";
      this->mesh_use_embedded_materials = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _ns_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ns_type ns;
  using _id_type =
    int32_t;
  _id_type id;
  using _type_type =
    int32_t;
  _type_type type;
  using _action_type =
    int32_t;
  _action_type action;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _scale_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _scale_type scale;
  using _color_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _color_type color;
  using _lifetime_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _lifetime_type lifetime;
  using _frame_locked_type =
    bool;
  _frame_locked_type frame_locked;
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;
  using _colors_type =
    std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::ColorRGBA_<ContainerAllocator>>>;
  _colors_type colors;
  using _texture_resource_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _texture_resource_type texture_resource;
  using _texture_type =
    sensor_msgs::msg::CompressedImage_<ContainerAllocator>;
  _texture_type texture;
  using _uv_coordinates_type =
    std::vector<visualization_msgs::msg::UVCoordinate_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::UVCoordinate_<ContainerAllocator>>>;
  _uv_coordinates_type uv_coordinates;
  using _text_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _text_type text;
  using _mesh_resource_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mesh_resource_type mesh_resource;
  using _mesh_file_type =
    visualization_msgs::msg::MeshFile_<ContainerAllocator>;
  _mesh_file_type mesh_file;
  using _mesh_use_embedded_materials_type =
    bool;
  _mesh_use_embedded_materials_type mesh_use_embedded_materials;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__ns(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ns = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__type(
    const int32_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__action(
    const int32_t & _arg)
  {
    this->action = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__scale(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->scale = _arg;
    return *this;
  }
  Type & set__color(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__lifetime(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->lifetime = _arg;
    return *this;
  }
  Type & set__frame_locked(
    const bool & _arg)
  {
    this->frame_locked = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__colors(
    const std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::ColorRGBA_<ContainerAllocator>>> & _arg)
  {
    this->colors = _arg;
    return *this;
  }
  Type & set__texture_resource(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->texture_resource = _arg;
    return *this;
  }
  Type & set__texture(
    const sensor_msgs::msg::CompressedImage_<ContainerAllocator> & _arg)
  {
    this->texture = _arg;
    return *this;
  }
  Type & set__uv_coordinates(
    const std::vector<visualization_msgs::msg::UVCoordinate_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::UVCoordinate_<ContainerAllocator>>> & _arg)
  {
    this->uv_coordinates = _arg;
    return *this;
  }
  Type & set__text(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->text = _arg;
    return *this;
  }
  Type & set__mesh_resource(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mesh_resource = _arg;
    return *this;
  }
  Type & set__mesh_file(
    const visualization_msgs::msg::MeshFile_<ContainerAllocator> & _arg)
  {
    this->mesh_file = _arg;
    return *this;
  }
  Type & set__mesh_use_embedded_materials(
    const bool & _arg)
  {
    this->mesh_use_embedded_materials = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t ARROW =
    0;
  static constexpr int32_t CUBE =
    1;
  static constexpr int32_t SPHERE =
    2;
  static constexpr int32_t CYLINDER =
    3;
  static constexpr int32_t LINE_STRIP =
    4;
  static constexpr int32_t LINE_LIST =
    5;
  static constexpr int32_t CUBE_LIST =
    6;
  static constexpr int32_t SPHERE_LIST =
    7;
  static constexpr int32_t POINTS =
    8;
  static constexpr int32_t TEXT_VIEW_FACING =
    9;
  static constexpr int32_t MESH_RESOURCE =
    10;
  static constexpr int32_t TRIANGLE_LIST =
    11;
  static constexpr int32_t ARROW_STRIP =
    12;
  static constexpr int32_t ADD =
    0;
  static constexpr int32_t MODIFY =
    0;
  // guard against 'DELETE' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(DELETE)
#    pragma push_macro("DELETE")
#    undef DELETE
#  endif
#endif
  static constexpr int32_t DELETE =
    2;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("DELETE")
#endif
  static constexpr int32_t DELETEALL =
    3;

  // pointer types
  using RawPtr =
    visualization_msgs::msg::Marker_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::Marker_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::Marker_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::Marker_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::Marker_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::Marker_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::Marker_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::Marker_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::Marker_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::Marker_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__Marker
    std::shared_ptr<visualization_msgs::msg::Marker_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__Marker
    std::shared_ptr<visualization_msgs::msg::Marker_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Marker_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->ns != other.ns) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->action != other.action) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->scale != other.scale) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    if (this->lifetime != other.lifetime) {
      return false;
    }
    if (this->frame_locked != other.frame_locked) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->colors != other.colors) {
      return false;
    }
    if (this->texture_resource != other.texture_resource) {
      return false;
    }
    if (this->texture != other.texture) {
      return false;
    }
    if (this->uv_coordinates != other.uv_coordinates) {
      return false;
    }
    if (this->text != other.text) {
      return false;
    }
    if (this->mesh_resource != other.mesh_resource) {
      return false;
    }
    if (this->mesh_file != other.mesh_file) {
      return false;
    }
    if (this->mesh_use_embedded_materials != other.mesh_use_embedded_materials) {
      return false;
    }
    return true;
  }
  bool operator!=(const Marker_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Marker_

// alias to use template instance with default allocator
using Marker =
  visualization_msgs::msg::Marker_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::ARROW;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::CUBE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::SPHERE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::CYLINDER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::LINE_STRIP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::LINE_LIST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::CUBE_LIST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::SPHERE_LIST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::POINTS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::TEXT_VIEW_FACING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::MESH_RESOURCE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::TRIANGLE_LIST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::ARROW_STRIP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::ADD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::MODIFY;
#endif  // __cplusplus < 201703L
// guard against 'DELETE' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(DELETE)
#    pragma push_macro("DELETE")
#    undef DELETE
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::DELETE;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("DELETE")
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Marker_<ContainerAllocator>::DELETEALL;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/InteractiveMarkerControl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/interactive_marker_control.hpp"


#ifndef DIMOS_CDR_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95
#define DIMOS_CDR_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'orientation'
// Member 'markers'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerControl __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerControl __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InteractiveMarkerControl_
{
  using Type = InteractiveMarkerControl_<ContainerAllocator>;
void validate() const {
orientation.validate();
for (const auto& item : markers) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerControl";

  explicit InteractiveMarkerControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : orientation(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->orientation_mode = 0;
      this->interaction_mode = 0;
      this->always_visible = false;
      this->independent_marker_orientation = false;
      this->description = "";
    }
  }

  explicit InteractiveMarkerControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    orientation(_alloc, _init),
    description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->orientation_mode = 0;
      this->interaction_mode = 0;
      this->always_visible = false;
      this->independent_marker_orientation = false;
      this->description = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _orientation_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _orientation_type orientation;
  using _orientation_mode_type =
    uint8_t;
  _orientation_mode_type orientation_mode;
  using _interaction_mode_type =
    uint8_t;
  _interaction_mode_type interaction_mode;
  using _always_visible_type =
    bool;
  _always_visible_type always_visible;
  using _markers_type =
    std::vector<visualization_msgs::msg::Marker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::Marker_<ContainerAllocator>>>;
  _markers_type markers;
  using _independent_marker_orientation_type =
    bool;
  _independent_marker_orientation_type independent_marker_orientation;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__orientation(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__orientation_mode(
    const uint8_t & _arg)
  {
    this->orientation_mode = _arg;
    return *this;
  }
  Type & set__interaction_mode(
    const uint8_t & _arg)
  {
    this->interaction_mode = _arg;
    return *this;
  }
  Type & set__always_visible(
    const bool & _arg)
  {
    this->always_visible = _arg;
    return *this;
  }
  Type & set__markers(
    const std::vector<visualization_msgs::msg::Marker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::Marker_<ContainerAllocator>>> & _arg)
  {
    this->markers = _arg;
    return *this;
  }
  Type & set__independent_marker_orientation(
    const bool & _arg)
  {
    this->independent_marker_orientation = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t INHERIT =
    0u;
  static constexpr uint8_t FIXED =
    1u;
  static constexpr uint8_t VIEW_FACING =
    2u;
  static constexpr uint8_t NONE =
    0u;
  static constexpr uint8_t MENU =
    1u;
  static constexpr uint8_t BUTTON =
    2u;
  static constexpr uint8_t MOVE_AXIS =
    3u;
  static constexpr uint8_t MOVE_PLANE =
    4u;
  static constexpr uint8_t ROTATE_AXIS =
    5u;
  static constexpr uint8_t MOVE_ROTATE =
    6u;
  static constexpr uint8_t MOVE_3D =
    7u;
  static constexpr uint8_t ROTATE_3D =
    8u;
  static constexpr uint8_t MOVE_ROTATE_3D =
    9u;

  // pointer types
  using RawPtr =
    visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerControl
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerControl
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InteractiveMarkerControl_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->orientation_mode != other.orientation_mode) {
      return false;
    }
    if (this->interaction_mode != other.interaction_mode) {
      return false;
    }
    if (this->always_visible != other.always_visible) {
      return false;
    }
    if (this->markers != other.markers) {
      return false;
    }
    if (this->independent_marker_orientation != other.independent_marker_orientation) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const InteractiveMarkerControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InteractiveMarkerControl_

// alias to use template instance with default allocator
using InteractiveMarkerControl =
  visualization_msgs::msg::InteractiveMarkerControl_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::INHERIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::FIXED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::VIEW_FACING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::MENU;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::BUTTON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::MOVE_AXIS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::MOVE_PLANE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::ROTATE_AXIS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::MOVE_ROTATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::MOVE_3D;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::ROTATE_3D;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerControl_<ContainerAllocator>::MOVE_ROTATE_3D;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/MenuEntry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/menu_entry.hpp"


#ifndef DIMOS_CDR_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618
#define DIMOS_CDR_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__MenuEntry __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__MenuEntry __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MenuEntry_
{
  using Type = MenuEntry_<ContainerAllocator>;
void validate() const {

}
static constexpr const char* msg_name = "visualization_msgs/msg/MenuEntry";

  explicit MenuEntry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->parent_id = 0ul;
      this->title = "";
      this->command = "";
      this->command_type = 0;
    }
  }

  explicit MenuEntry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : title(_alloc),
    command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->parent_id = 0ul;
      this->title = "";
      this->command = "";
      this->command_type = 0;
    }
  }

  // field types and members
  using _id_type =
    uint32_t;
  _id_type id;
  using _parent_id_type =
    uint32_t;
  _parent_id_type parent_id;
  using _title_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _title_type title;
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;
  using _command_type_type =
    uint8_t;
  _command_type_type command_type;

  // setters for named parameter idiom
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__parent_id(
    const uint32_t & _arg)
  {
    this->parent_id = _arg;
    return *this;
  }
  Type & set__title(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->title = _arg;
    return *this;
  }
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__command_type(
    const uint8_t & _arg)
  {
    this->command_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FEEDBACK =
    0u;
  static constexpr uint8_t ROSRUN =
    1u;
  static constexpr uint8_t ROSLAUNCH =
    2u;

  // pointer types
  using RawPtr =
    visualization_msgs::msg::MenuEntry_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::MenuEntry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::MenuEntry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::MenuEntry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::MenuEntry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::MenuEntry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::MenuEntry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::MenuEntry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::MenuEntry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::MenuEntry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__MenuEntry
    std::shared_ptr<visualization_msgs::msg::MenuEntry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__MenuEntry
    std::shared_ptr<visualization_msgs::msg::MenuEntry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MenuEntry_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->parent_id != other.parent_id) {
      return false;
    }
    if (this->title != other.title) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->command_type != other.command_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const MenuEntry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MenuEntry_

// alias to use template instance with default allocator
using MenuEntry =
  visualization_msgs::msg::MenuEntry_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MenuEntry_<ContainerAllocator>::FEEDBACK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MenuEntry_<ContainerAllocator>::ROSRUN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MenuEntry_<ContainerAllocator>::ROSLAUNCH;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/InteractiveMarker.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/interactive_marker.hpp"


#ifndef DIMOS_CDR_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847
#define DIMOS_CDR_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'pose'
// Member 'menu_entries'
// Member 'controls'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__InteractiveMarker __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__InteractiveMarker __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InteractiveMarker_
{
  using Type = InteractiveMarker_<ContainerAllocator>;
void validate() const {
header.validate();
pose.validate();
for (const auto& item : menu_entries) { item.validate(); }
for (const auto& item : controls) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarker";

  explicit InteractiveMarker_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->description = "";
      this->scale = 0.0f;
    }
  }

  explicit InteractiveMarker_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    name(_alloc),
    description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->description = "";
      this->scale = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;
  using _scale_type =
    float;
  _scale_type scale;
  using _menu_entries_type =
    std::vector<visualization_msgs::msg::MenuEntry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::MenuEntry_<ContainerAllocator>>>;
  _menu_entries_type menu_entries;
  using _controls_type =
    std::vector<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>>>;
  _controls_type controls;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }
  Type & set__scale(
    const float & _arg)
  {
    this->scale = _arg;
    return *this;
  }
  Type & set__menu_entries(
    const std::vector<visualization_msgs::msg::MenuEntry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::MenuEntry_<ContainerAllocator>>> & _arg)
  {
    this->menu_entries = _arg;
    return *this;
  }
  Type & set__controls(
    const std::vector<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::InteractiveMarkerControl_<ContainerAllocator>>> & _arg)
  {
    this->controls = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    visualization_msgs::msg::InteractiveMarker_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::InteractiveMarker_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarker
    std::shared_ptr<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarker
    std::shared_ptr<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InteractiveMarker_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    if (this->scale != other.scale) {
      return false;
    }
    if (this->menu_entries != other.menu_entries) {
      return false;
    }
    if (this->controls != other.controls) {
      return false;
    }
    return true;
  }
  bool operator!=(const InteractiveMarker_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InteractiveMarker_

// alias to use template instance with default allocator
using InteractiveMarker =
  visualization_msgs::msg::InteractiveMarker_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/InteractiveMarkerFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/interactive_marker_feedback.hpp"


#ifndef DIMOS_CDR_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E
#define DIMOS_CDR_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'pose'
// Member 'mouse_point'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerFeedback __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerFeedback __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InteractiveMarkerFeedback_
{
  using Type = InteractiveMarkerFeedback_<ContainerAllocator>;
void validate() const {
header.validate();
pose.validate();
mouse_point.validate();
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerFeedback";

  explicit InteractiveMarkerFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    mouse_point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->client_id = "";
      this->marker_name = "";
      this->control_name = "";
      this->event_type = 0;
      this->menu_entry_id = 0ul;
      this->mouse_point_valid = false;
    }
  }

  explicit InteractiveMarkerFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    client_id(_alloc),
    marker_name(_alloc),
    control_name(_alloc),
    pose(_alloc, _init),
    mouse_point(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->client_id = "";
      this->marker_name = "";
      this->control_name = "";
      this->event_type = 0;
      this->menu_entry_id = 0ul;
      this->mouse_point_valid = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _client_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _client_id_type client_id;
  using _marker_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _marker_name_type marker_name;
  using _control_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _control_name_type control_name;
  using _event_type_type =
    uint8_t;
  _event_type_type event_type;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _menu_entry_id_type =
    uint32_t;
  _menu_entry_id_type menu_entry_id;
  using _mouse_point_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _mouse_point_type mouse_point;
  using _mouse_point_valid_type =
    bool;
  _mouse_point_valid_type mouse_point_valid;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__client_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->client_id = _arg;
    return *this;
  }
  Type & set__marker_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->marker_name = _arg;
    return *this;
  }
  Type & set__control_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->control_name = _arg;
    return *this;
  }
  Type & set__event_type(
    const uint8_t & _arg)
  {
    this->event_type = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__menu_entry_id(
    const uint32_t & _arg)
  {
    this->menu_entry_id = _arg;
    return *this;
  }
  Type & set__mouse_point(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->mouse_point = _arg;
    return *this;
  }
  Type & set__mouse_point_valid(
    const bool & _arg)
  {
    this->mouse_point_valid = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t KEEP_ALIVE =
    0u;
  static constexpr uint8_t POSE_UPDATE =
    1u;
  static constexpr uint8_t MENU_SELECT =
    2u;
  static constexpr uint8_t BUTTON_CLICK =
    3u;
  static constexpr uint8_t MOUSE_DOWN =
    4u;
  static constexpr uint8_t MOUSE_UP =
    5u;

  // pointer types
  using RawPtr =
    visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerFeedback
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerFeedback
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InteractiveMarkerFeedback_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->client_id != other.client_id) {
      return false;
    }
    if (this->marker_name != other.marker_name) {
      return false;
    }
    if (this->control_name != other.control_name) {
      return false;
    }
    if (this->event_type != other.event_type) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->menu_entry_id != other.menu_entry_id) {
      return false;
    }
    if (this->mouse_point != other.mouse_point) {
      return false;
    }
    if (this->mouse_point_valid != other.mouse_point_valid) {
      return false;
    }
    return true;
  }
  bool operator!=(const InteractiveMarkerFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InteractiveMarkerFeedback_

// alias to use template instance with default allocator
using InteractiveMarkerFeedback =
  visualization_msgs::msg::InteractiveMarkerFeedback_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::KEEP_ALIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::POSE_UPDATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::MENU_SELECT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::BUTTON_CLICK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::MOUSE_DOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerFeedback_<ContainerAllocator>::MOUSE_UP;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/InteractiveMarkerInit.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/interactive_marker_init.hpp"


#ifndef DIMOS_CDR_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4
#define DIMOS_CDR_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'markers'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerInit __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerInit __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InteractiveMarkerInit_
{
  using Type = InteractiveMarkerInit_<ContainerAllocator>;
void validate() const {
for (const auto& item : markers) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerInit";

  explicit InteractiveMarkerInit_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->server_id = "";
      this->seq_num = 0ull;
    }
  }

  explicit InteractiveMarkerInit_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : server_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->server_id = "";
      this->seq_num = 0ull;
    }
  }

  // field types and members
  using _server_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _server_id_type server_id;
  using _seq_num_type =
    uint64_t;
  _seq_num_type seq_num;
  using _markers_type =
    std::vector<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>>;
  _markers_type markers;

  // setters for named parameter idiom
  Type & set__server_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->server_id = _arg;
    return *this;
  }
  Type & set__seq_num(
    const uint64_t & _arg)
  {
    this->seq_num = _arg;
    return *this;
  }
  Type & set__markers(
    const std::vector<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>> & _arg)
  {
    this->markers = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerInit
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerInit
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerInit_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InteractiveMarkerInit_ & other) const
  {
    if (this->server_id != other.server_id) {
      return false;
    }
    if (this->seq_num != other.seq_num) {
      return false;
    }
    if (this->markers != other.markers) {
      return false;
    }
    return true;
  }
  bool operator!=(const InteractiveMarkerInit_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InteractiveMarkerInit_

// alias to use template instance with default allocator
using InteractiveMarkerInit =
  visualization_msgs::msg::InteractiveMarkerInit_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/InteractiveMarkerPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/interactive_marker_pose.hpp"


#ifndef DIMOS_CDR_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5
#define DIMOS_CDR_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'header'
// Member 'pose'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerPose __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerPose __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InteractiveMarkerPose_
{
  using Type = InteractiveMarkerPose_<ContainerAllocator>;
void validate() const {
header.validate();
pose.validate();
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerPose";

  explicit InteractiveMarkerPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  explicit InteractiveMarkerPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerPose
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerPose
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InteractiveMarkerPose_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    return true;
  }
  bool operator!=(const InteractiveMarkerPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InteractiveMarkerPose_

// alias to use template instance with default allocator
using InteractiveMarkerPose =
  visualization_msgs::msg::InteractiveMarkerPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/InteractiveMarkerUpdate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/interactive_marker_update.hpp"


#ifndef DIMOS_CDR_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5
#define DIMOS_CDR_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'markers'
// Member 'poses'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerUpdate __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__InteractiveMarkerUpdate __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InteractiveMarkerUpdate_
{
  using Type = InteractiveMarkerUpdate_<ContainerAllocator>;
void validate() const {
for (const auto& item : markers) { item.validate(); }
for (const auto& item : poses) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/InteractiveMarkerUpdate";

  explicit InteractiveMarkerUpdate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->server_id = "";
      this->seq_num = 0ull;
      this->type = 0;
    }
  }

  explicit InteractiveMarkerUpdate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : server_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->server_id = "";
      this->seq_num = 0ull;
      this->type = 0;
    }
  }

  // field types and members
  using _server_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _server_id_type server_id;
  using _seq_num_type =
    uint64_t;
  _seq_num_type seq_num;
  using _type_type =
    uint8_t;
  _type_type type;
  using _markers_type =
    std::vector<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>>;
  _markers_type markers;
  using _poses_type =
    std::vector<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>>>;
  _poses_type poses;
  using _erases_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _erases_type erases;

  // setters for named parameter idiom
  Type & set__server_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->server_id = _arg;
    return *this;
  }
  Type & set__seq_num(
    const uint64_t & _arg)
  {
    this->seq_num = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__markers(
    const std::vector<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::InteractiveMarker_<ContainerAllocator>>> & _arg)
  {
    this->markers = _arg;
    return *this;
  }
  Type & set__poses(
    const std::vector<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::InteractiveMarkerPose_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }
  Type & set__erases(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->erases = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t KEEP_ALIVE =
    0u;
  static constexpr uint8_t UPDATE =
    1u;

  // pointer types
  using RawPtr =
    visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerUpdate
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__InteractiveMarkerUpdate
    std::shared_ptr<visualization_msgs::msg::InteractiveMarkerUpdate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InteractiveMarkerUpdate_ & other) const
  {
    if (this->server_id != other.server_id) {
      return false;
    }
    if (this->seq_num != other.seq_num) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->markers != other.markers) {
      return false;
    }
    if (this->poses != other.poses) {
      return false;
    }
    if (this->erases != other.erases) {
      return false;
    }
    return true;
  }
  bool operator!=(const InteractiveMarkerUpdate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InteractiveMarkerUpdate_

// alias to use template instance with default allocator
using InteractiveMarkerUpdate =
  visualization_msgs::msg::InteractiveMarkerUpdate_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerUpdate_<ContainerAllocator>::KEEP_ALIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t InteractiveMarkerUpdate_<ContainerAllocator>::UPDATE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visualization_msgs:msg/MarkerArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "visualization_msgs/msg/marker_array.hpp"


#ifndef DIMOS_CDR_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22
#define DIMOS_CDR_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>



// Include directives for member types
// Member 'markers'

#ifndef _WIN32
# define DEPRECATED__visualization_msgs__msg__MarkerArray __attribute__((deprecated))
#else
# define DEPRECATED__visualization_msgs__msg__MarkerArray __declspec(deprecated)
#endif

namespace visualization_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MarkerArray_
{
  using Type = MarkerArray_<ContainerAllocator>;
void validate() const {
for (const auto& item : markers) { item.validate(); }
}
static constexpr const char* msg_name = "visualization_msgs/msg/MarkerArray";

  explicit MarkerArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MarkerArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _markers_type =
    std::vector<visualization_msgs::msg::Marker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::Marker_<ContainerAllocator>>>;
  _markers_type markers;

  // setters for named parameter idiom
  Type & set__markers(
    const std::vector<visualization_msgs::msg::Marker_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visualization_msgs::msg::Marker_<ContainerAllocator>>> & _arg)
  {
    this->markers = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    visualization_msgs::msg::MarkerArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const visualization_msgs::msg::MarkerArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visualization_msgs::msg::MarkerArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visualization_msgs::msg::MarkerArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::MarkerArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::MarkerArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visualization_msgs::msg::MarkerArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visualization_msgs::msg::MarkerArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visualization_msgs::msg::MarkerArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visualization_msgs::msg::MarkerArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visualization_msgs__msg__MarkerArray
    std::shared_ptr<visualization_msgs::msg::MarkerArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visualization_msgs__msg__MarkerArray
    std::shared_ptr<visualization_msgs::msg::MarkerArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarkerArray_ & other) const
  {
    if (this->markers != other.markers) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarkerArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarkerArray_

// alias to use template instance with default allocator
using MarkerArray =
  visualization_msgs::msg::MarkerArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace visualization_msgs

#endif  // DIMOS_CDR_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22
namespace eprosima::fastcdr {
#ifndef DIMOS_CDR_BOUNDED_E9D8D5F8FC819DF85243FD7F0F1CF04F5FDE666712292CD129F01DB36550F43F
#define DIMOS_CDR_BOUNDED_E9D8D5F8FC819DF85243FD7F0F1CF04F5FDE666712292CD129F01DB36550F43F
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& c, const shape_msgs::msg::SolidPrimitive::_dimensions_type& v, size_t& a) { return c.calculate_serialized_size(std::vector<double>(v.begin(), v.end()), a); }
template<> inline void serialize(Cdr& c, const shape_msgs::msg::SolidPrimitive::_dimensions_type& v) { c << std::vector<double>(v.begin(), v.end()); }
template<> inline void deserialize(Cdr& c, shape_msgs::msg::SolidPrimitive::_dimensions_type& v) { std::vector<double> values; c >> values; v.assign(values.begin(), values.end()); }
#endif
#ifndef DIMOS_MESSAGE_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C_CODEC
#define DIMOS_MESSAGE_3AC0DB8DD9699222174D1DAED52F7ECA3ACF16C531D88ED55CD7A0AE9CE20D5C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const builtin_interfaces::msg::Duration& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.sec, alignment);
size += calculator.calculate_serialized_size(value.nanosec, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const builtin_interfaces::msg::Duration& value) {
cdr << value.sec;
cdr << value.nanosec;
}
template<> inline void deserialize(Cdr& cdr, builtin_interfaces::msg::Duration& value) {
cdr >> value.sec;
cdr >> value.nanosec;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4_CODEC
#define DIMOS_MESSAGE_6F3F28F5724CDFCB39E91219BE17457BCCE4B7CB1B65E9427A7F640F688ABFC4_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const builtin_interfaces::msg::Time& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.sec, alignment);
size += calculator.calculate_serialized_size(value.nanosec, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const builtin_interfaces::msg::Time& value) {
cdr << value.sec;
cdr << value.nanosec;
}
template<> inline void deserialize(Cdr& cdr, builtin_interfaces::msg::Time& value) {
cdr >> value.sec;
cdr >> value.nanosec;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D_CODEC
#define DIMOS_MESSAGE_D233189694BAC337E6192CD32A8C3457C3D57F2FAADA7C6D5F86E9A3712D451D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Header& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.stamp, alignment);
size += calculator.calculate_serialized_size(value.frame_id, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Header& value) {
cdr << value.stamp;
cdr << value.frame_id;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Header& value) {
cdr >> value.stamp;
cdr >> value.frame_id;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51_CODEC
#define DIMOS_MESSAGE_54FD6FB210AB8E1531B4E06D08873EAB05A1EC45052903EE83320A92E2683F51_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Point2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Point2D& value) {
cdr << value.x;
cdr << value.y;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Point2D& value) {
cdr >> value.x;
cdr >> value.y;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97_CODEC
#define DIMOS_MESSAGE_DD9FAC5B16FD54B1ADE2D63BFA6C5EF0A4C57BD436102D2BF2B938C20A25CB97_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Pose2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.position, alignment);
size += calculator.calculate_serialized_size(value.theta, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Pose2D& value) {
cdr << value.position;
cdr << value.theta;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Pose2D& value) {
cdr >> value.position;
cdr >> value.theta;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA_CODEC
#define DIMOS_MESSAGE_753D94893DEF396D6E1F2F0A19ECA6D196F5AA5E260B9EB63FB71DE87A782FFA_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::BoundingBox2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.center, alignment);
size += calculator.calculate_serialized_size(value.size_x, alignment);
size += calculator.calculate_serialized_size(value.size_y, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::BoundingBox2D& value) {
cdr << value.center;
cdr << value.size_x;
cdr << value.size_y;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::BoundingBox2D& value) {
cdr >> value.center;
cdr >> value.size_x;
cdr >> value.size_y;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A_CODEC
#define DIMOS_MESSAGE_94E8B3D10CF2F89103470F4DBC02B19CF7A8EBD9D8CE71328FA1CDDB9CFDA94A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::BoundingBox2DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.boxes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::BoundingBox2DArray& value) {
cdr << value.header;
cdr << value.boxes;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::BoundingBox2DArray& value) {
cdr >> value.header;
cdr >> value.boxes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C_CODEC
#define DIMOS_MESSAGE_778B613D0D80A56FDBCB3399735EEBFC59783D5288C5EFEC12B1B3F68050D80C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Point& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.z, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Point& value) {
cdr << value.x;
cdr << value.y;
cdr << value.z;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Point& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.z;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD_CODEC
#define DIMOS_MESSAGE_1876AC8F11336F2526036B2809EA73FCC1BF298514D05209C22FFFAE10E08EFD_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Quaternion& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.z, alignment);
size += calculator.calculate_serialized_size(value.w, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Quaternion& value) {
cdr << value.x;
cdr << value.y;
cdr << value.z;
cdr << value.w;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Quaternion& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.z;
cdr >> value.w;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827_CODEC
#define DIMOS_MESSAGE_5825AE7A15EA8E533DEF906B88D079A716D092836CCA13CA1E823199910BA827_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Pose& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.position, alignment);
size += calculator.calculate_serialized_size(value.orientation, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Pose& value) {
cdr << value.position;
cdr << value.orientation;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Pose& value) {
cdr >> value.position;
cdr >> value.orientation;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02_CODEC
#define DIMOS_MESSAGE_ED5BD99AB762FB6B65CE4D31256826B1EB52AC1FF931AAAB8D4FDCCC3C945B02_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Vector3& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.z, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Vector3& value) {
cdr << value.x;
cdr << value.y;
cdr << value.z;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Vector3& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.z;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0_CODEC
#define DIMOS_MESSAGE_05C72D3B9590295997A376972262E6C50E3C8C08048670CEDA9F85910050F4F0_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::BoundingBox3D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.center, alignment);
size += calculator.calculate_serialized_size(value.size, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::BoundingBox3D& value) {
cdr << value.center;
cdr << value.size;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::BoundingBox3D& value) {
cdr >> value.center;
cdr >> value.size;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A_CODEC
#define DIMOS_MESSAGE_B0DA6AEABAC47CA9A469EFECC4A3132770379461DD6D11CE59897F3C987D707A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::BoundingBox3DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.boxes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::BoundingBox3DArray& value) {
cdr << value.header;
cdr << value.boxes;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::BoundingBox3DArray& value) {
cdr >> value.header;
cdr >> value.boxes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87_CODEC
#define DIMOS_MESSAGE_75CDD22DA5250DF14241A62DB00DA66785925A772BE3CE5EFB64D463D9D84E87_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::EntityMarker& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.entity_id, alignment);
size += calculator.calculate_serialized_size(value.label, alignment);
size += calculator.calculate_serialized_size(value.entity_type, alignment);
size += calculator.calculate_serialized_size(value.position, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::EntityMarker& value) {
cdr << value.entity_id;
cdr << value.label;
cdr << value.entity_type;
cdr << value.position;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::EntityMarker& value) {
cdr >> value.entity_id;
cdr >> value.label;
cdr >> value.entity_type;
cdr >> value.position;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254_CODEC
#define DIMOS_MESSAGE_D16384B577AC64C3D9C281693C54BEA612EAF066B22C1525786C791F273C9254_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::EntityMarkers& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.markers, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::EntityMarkers& value) {
cdr << value.header;
cdr << value.markers;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::EntityMarkers& value) {
cdr >> value.header;
cdr >> value.markers;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6_CODEC
#define DIMOS_MESSAGE_73822A2CF3903B0E42CAB4C29A13C75CC99C347178F391D1CEE7AF631942F8F6_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::GraspCandidate& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.score, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::GraspCandidate& value) {
cdr << value.pose;
cdr << value.score;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::GraspCandidate& value) {
cdr >> value.pose;
cdr >> value.score;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC_CODEC
#define DIMOS_MESSAGE_A4E1A634054549284F3FB0006D65B3A42185856FABD0519D62C4678360EFCCAC_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::GraspCandidateArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.candidates, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::GraspCandidateArray& value) {
cdr << value.header;
cdr << value.candidates;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::GraspCandidateArray& value) {
cdr >> value.header;
cdr >> value.candidates;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533_CODEC
#define DIMOS_MESSAGE_F080E1984D729B29DAED633F1B188F70EBB76A5B10FD03C47A0BED1A76090533_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::ImuInfo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.gyro_noise_density, alignment);
size += calculator.calculate_serialized_size(value.gyro_random_walk, alignment);
size += calculator.calculate_serialized_size(value.accel_noise_density, alignment);
size += calculator.calculate_serialized_size(value.accel_random_walk, alignment);
size += calculator.calculate_serialized_size(value.frequency, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::ImuInfo& value) {
cdr << value.header;
cdr << value.gyro_noise_density;
cdr << value.gyro_random_walk;
cdr << value.accel_noise_density;
cdr << value.accel_random_walk;
cdr << value.frequency;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::ImuInfo& value) {
cdr >> value.header;
cdr >> value.gyro_noise_density;
cdr >> value.gyro_random_walk;
cdr >> value.accel_noise_density;
cdr >> value.accel_random_walk;
cdr >> value.frequency;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02_CODEC
#define DIMOS_MESSAGE_487536BCD194072C56FA0D805BE248F4BF992A3CF96247502E4FF3992D1C1F02_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::JointCommand& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.positions, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::JointCommand& value) {
cdr << value.header;
cdr << value.positions;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::JointCommand& value) {
cdr >> value.header;
cdr >> value.positions;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2_CODEC
#define DIMOS_MESSAGE_E17EA2A74FDDC0CEA071C038B9C7F8E9A340B8048B78CBFC50D0DACAB4B75AE2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::LineSegment3D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.start, alignment);
size += calculator.calculate_serialized_size(value.end, alignment);
size += calculator.calculate_serialized_size(value.weight, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::LineSegment3D& value) {
cdr << value.start;
cdr << value.end;
cdr << value.weight;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::LineSegment3D& value) {
cdr >> value.start;
cdr >> value.end;
cdr >> value.weight;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C_CODEC
#define DIMOS_MESSAGE_4B583B0205E607E83DF19B6662F10FA00C3FAEB85123B078C92C299B1CB3C07C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::LineSegments3D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.segments, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::LineSegments3D& value) {
cdr << value.header;
cdr << value.segments;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::LineSegments3D& value) {
cdr >> value.header;
cdr >> value.segments;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03_CODEC
#define DIMOS_MESSAGE_CC750E77F5D3AAB17F990FDE4A41A59031D260819C4A642940789051EA443D03_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::MotorCommandArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.q, alignment);
size += calculator.calculate_serialized_size(value.dq, alignment);
size += calculator.calculate_serialized_size(value.kp, alignment);
size += calculator.calculate_serialized_size(value.kd, alignment);
size += calculator.calculate_serialized_size(value.tau, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::MotorCommandArray& value) {
cdr << value.header;
cdr << value.q;
cdr << value.dq;
cdr << value.kp;
cdr << value.kd;
cdr << value.tau;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::MotorCommandArray& value) {
cdr >> value.header;
cdr >> value.q;
cdr >> value.dq;
cdr >> value.kp;
cdr >> value.kd;
cdr >> value.tau;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3_CODEC
#define DIMOS_MESSAGE_546CF5D6DB927E3D3BE5871556F6231CBDB5C769B1F7018302EAE2A6CC8B00B3_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::RobotState& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.state, alignment);
size += calculator.calculate_serialized_size(value.mode, alignment);
size += calculator.calculate_serialized_size(value.error_code, alignment);
size += calculator.calculate_serialized_size(value.warn_code, alignment);
size += calculator.calculate_serialized_size(value.cmdnum, alignment);
size += calculator.calculate_serialized_size(value.mt_brake, alignment);
size += calculator.calculate_serialized_size(value.mt_able, alignment);
size += calculator.calculate_serialized_size(value.tcp_pose, alignment);
size += calculator.calculate_serialized_size(value.tcp_offset, alignment);
size += calculator.calculate_serialized_size(value.joints, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::RobotState& value) {
cdr << value.header;
cdr << value.state;
cdr << value.mode;
cdr << value.error_code;
cdr << value.warn_code;
cdr << value.cmdnum;
cdr << value.mt_brake;
cdr << value.mt_able;
cdr << value.tcp_pose;
cdr << value.tcp_offset;
cdr << value.joints;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::RobotState& value) {
cdr >> value.header;
cdr >> value.state;
cdr >> value.mode;
cdr >> value.error_code;
cdr >> value.warn_code;
cdr >> value.cmdnum;
cdr >> value.mt_brake;
cdr >> value.mt_able;
cdr >> value.tcp_pose;
cdr >> value.tcp_offset;
cdr >> value.joints;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64_CODEC
#define DIMOS_MESSAGE_D7DF4280E9389BCF868E804194944510CDDC42C6F6B3B59BAFD864A0D0869D64_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::TrajectoryStatus& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.state, alignment);
size += calculator.calculate_serialized_size(value.progress, alignment);
size += calculator.calculate_serialized_size(value.time_elapsed, alignment);
size += calculator.calculate_serialized_size(value.time_remaining, alignment);
size += calculator.calculate_serialized_size(value.error, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::TrajectoryStatus& value) {
cdr << value.header;
cdr << value.state;
cdr << value.progress;
cdr << value.time_elapsed;
cdr << value.time_remaining;
cdr << value.error;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::TrajectoryStatus& value) {
cdr >> value.header;
cdr >> value.state;
cdr >> value.progress;
cdr >> value.time_elapsed;
cdr >> value.time_remaining;
cdr >> value.error;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125_CODEC
#define DIMOS_MESSAGE_CEA7983751CAA68548C19225E57F04FA263D029ABC833D9E8A26D6B599051125_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const dimos_msgs::msg::VideoStats& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.fps, alignment);
size += calculator.calculate_serialized_size(value.kbps, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.loss_pct, alignment);
size += calculator.calculate_serialized_size(value.jitter_buffer_ms, alignment);
size += calculator.calculate_serialized_size(value.decode_ms, alignment);
size += calculator.calculate_serialized_size(value.frames_dropped, alignment);
size += calculator.calculate_serialized_size(value.freezes, alignment);
size += calculator.calculate_serialized_size(value.e2e_latency_ms, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const dimos_msgs::msg::VideoStats& value) {
cdr << value.header;
cdr << value.fps;
cdr << value.kbps;
cdr << value.width;
cdr << value.height;
cdr << value.loss_pct;
cdr << value.jitter_buffer_ms;
cdr << value.decode_ms;
cdr << value.frames_dropped;
cdr << value.freezes;
cdr << value.e2e_latency_ms;
}
template<> inline void deserialize(Cdr& cdr, dimos_msgs::msg::VideoStats& value) {
cdr >> value.header;
cdr >> value.fps;
cdr >> value.kbps;
cdr >> value.width;
cdr >> value.height;
cdr >> value.loss_pct;
cdr >> value.jitter_buffer_ms;
cdr >> value.decode_ms;
cdr >> value.frames_dropped;
cdr >> value.freezes;
cdr >> value.e2e_latency_ms;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82_CODEC
#define DIMOS_MESSAGE_65E130BD9C02FCDC97D01DF27872E1CFD0BABE819FA6371D85CF5A268B352E82_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const foxglove_msgs::msg::CompressedVideo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.timestamp, alignment);
size += calculator.calculate_serialized_size(value.frame_id, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
size += calculator.calculate_serialized_size(value.format, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const foxglove_msgs::msg::CompressedVideo& value) {
cdr << value.timestamp;
cdr << value.frame_id;
cdr << value.data;
cdr << value.format;
}
template<> inline void deserialize(Cdr& cdr, foxglove_msgs::msg::CompressedVideo& value) {
cdr >> value.timestamp;
cdr >> value.frame_id;
cdr >> value.data;
cdr >> value.format;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962_CODEC
#define DIMOS_MESSAGE_9C1ACB3FBDFCD8FE69FB96B7C1CA90404FAF4E074E29210F3985FBD795CDA962_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Accel& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.linear, alignment);
size += calculator.calculate_serialized_size(value.angular, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Accel& value) {
cdr << value.linear;
cdr << value.angular;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Accel& value) {
cdr >> value.linear;
cdr >> value.angular;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3_CODEC
#define DIMOS_MESSAGE_1A5DABD6AC007D254B0D523A7C9F0E3EAB61AE6EB6CFCA3FBF9A9FDC11F7AAB3_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::AccelStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.accel, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::AccelStamped& value) {
cdr << value.header;
cdr << value.accel;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::AccelStamped& value) {
cdr >> value.header;
cdr >> value.accel;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7_CODEC
#define DIMOS_MESSAGE_F7D90F572A0F5A974976AC5E49EFAD0D841D3C5E6E0DFDC39421B3C73D38EDC7_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::AccelWithCovariance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.accel, alignment);
size += calculator.calculate_serialized_size(value.covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::AccelWithCovariance& value) {
cdr << value.accel;
cdr << value.covariance;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::AccelWithCovariance& value) {
cdr >> value.accel;
cdr >> value.covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283_CODEC
#define DIMOS_MESSAGE_ABE8C23F2F89EE686C07DBE3C0F1F9A1019D88D6EE6CD30D1DB3390983C7E283_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::AccelWithCovarianceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.accel, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::AccelWithCovarianceStamped& value) {
cdr << value.header;
cdr << value.accel;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::AccelWithCovarianceStamped& value) {
cdr >> value.header;
cdr >> value.accel;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE_CODEC
#define DIMOS_MESSAGE_ED3E961F94A7BE6A52BE2E5B43CE192E06F4B1A70C7076339424759D41196DBE_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Inertia& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.m, alignment);
size += calculator.calculate_serialized_size(value.com, alignment);
size += calculator.calculate_serialized_size(value.ixx, alignment);
size += calculator.calculate_serialized_size(value.ixy, alignment);
size += calculator.calculate_serialized_size(value.ixz, alignment);
size += calculator.calculate_serialized_size(value.iyy, alignment);
size += calculator.calculate_serialized_size(value.iyz, alignment);
size += calculator.calculate_serialized_size(value.izz, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Inertia& value) {
cdr << value.m;
cdr << value.com;
cdr << value.ixx;
cdr << value.ixy;
cdr << value.ixz;
cdr << value.iyy;
cdr << value.iyz;
cdr << value.izz;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Inertia& value) {
cdr >> value.m;
cdr >> value.com;
cdr >> value.ixx;
cdr >> value.ixy;
cdr >> value.ixz;
cdr >> value.iyy;
cdr >> value.iyz;
cdr >> value.izz;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF_CODEC
#define DIMOS_MESSAGE_1CD71210108FE040251DB26AFEB155D3276F07F42DB81BB7469FB36B12D832EF_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::InertiaStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.inertia, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::InertiaStamped& value) {
cdr << value.header;
cdr << value.inertia;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::InertiaStamped& value) {
cdr >> value.header;
cdr >> value.inertia;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6_CODEC
#define DIMOS_MESSAGE_6C0579A722E63D22C5659C730C0BC7D56B53FCA8450CB14CE8ED057AA431F1E6_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Point32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.z, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Point32& value) {
cdr << value.x;
cdr << value.y;
cdr << value.z;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Point32& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.z;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328_CODEC
#define DIMOS_MESSAGE_4BBD5748B98F3D83C2B32101053660A7CFF2A3701B5E84CEA5B7E76D1BE9E328_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PointStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.point, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PointStamped& value) {
cdr << value.header;
cdr << value.point;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PointStamped& value) {
cdr >> value.header;
cdr >> value.point;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469_CODEC
#define DIMOS_MESSAGE_E50882C172703452C54AAB048596E21FF3B1CA909B7A0B2CD197000F2F4BB469_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Polygon& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.points, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Polygon& value) {
cdr << value.points;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Polygon& value) {
cdr >> value.points;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C_CODEC
#define DIMOS_MESSAGE_DD3051A713EDB1A7B8158C32C4F44DE32913F2242F39DEF61D74DC0FC1B41E5C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PolygonInstance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.polygon, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PolygonInstance& value) {
cdr << value.polygon;
cdr << value.id;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PolygonInstance& value) {
cdr >> value.polygon;
cdr >> value.id;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22_CODEC
#define DIMOS_MESSAGE_7B176044B08CBD5EF5367566557C0D41CFFD9C685B8781987A31141E86DA1F22_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PolygonInstanceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.polygon, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PolygonInstanceStamped& value) {
cdr << value.header;
cdr << value.polygon;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PolygonInstanceStamped& value) {
cdr >> value.header;
cdr >> value.polygon;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F_CODEC
#define DIMOS_MESSAGE_6AE1AF0D73CA5397597ADFD5833DE81091ED5C0B0C2A97BB2A654B696B00208F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PolygonStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.polygon, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PolygonStamped& value) {
cdr << value.header;
cdr << value.polygon;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PolygonStamped& value) {
cdr >> value.header;
cdr >> value.polygon;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8_CODEC
#define DIMOS_MESSAGE_BBD3EBAC4CE3E7575D9A83BA0B7FA009CB86E4E198EFBC83333E673E9EAC43F8_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Pose2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x, alignment);
size += calculator.calculate_serialized_size(value.y, alignment);
size += calculator.calculate_serialized_size(value.theta, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Pose2D& value) {
cdr << value.x;
cdr << value.y;
cdr << value.theta;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Pose2D& value) {
cdr >> value.x;
cdr >> value.y;
cdr >> value.theta;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3_CODEC
#define DIMOS_MESSAGE_4585830E75FBF95DD419E0428C1486C5B8C97410CE1D1C3CA9957F3059D558B3_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PoseArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.poses, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PoseArray& value) {
cdr << value.header;
cdr << value.poses;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PoseArray& value) {
cdr >> value.header;
cdr >> value.poses;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F_CODEC
#define DIMOS_MESSAGE_F22E46D16557E898A6797FBF9A8616839671F0253AC498897E7C61446C01F65F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PoseStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PoseStamped& value) {
cdr << value.header;
cdr << value.pose;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PoseStamped& value) {
cdr >> value.header;
cdr >> value.pose;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A_CODEC
#define DIMOS_MESSAGE_B2A8A882D05FCABE341D870F7EA4D50EBD23110F7514D452EEC5E7BB1946F92A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PoseWithCovariance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PoseWithCovariance& value) {
cdr << value.pose;
cdr << value.covariance;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PoseWithCovariance& value) {
cdr >> value.pose;
cdr >> value.covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B_CODEC
#define DIMOS_MESSAGE_2E7458551623FC29FBA91890BBDB83B056023E2FA59A0C0566DAC4B982510C9B_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::PoseWithCovarianceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::PoseWithCovarianceStamped& value) {
cdr << value.header;
cdr << value.pose;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::PoseWithCovarianceStamped& value) {
cdr >> value.header;
cdr >> value.pose;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5_CODEC
#define DIMOS_MESSAGE_5103A3865A8742C15EE9A4FFA3083EB5428052E86DCE4EFC4F5E31498BA483C5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::QuaternionStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.quaternion, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::QuaternionStamped& value) {
cdr << value.header;
cdr << value.quaternion;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::QuaternionStamped& value) {
cdr >> value.header;
cdr >> value.quaternion;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238_CODEC
#define DIMOS_MESSAGE_D9B3A531152ADC15D692C321771BFD2E539C90E81FB362CA4E2EAB5BB6FBC238_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Transform& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.translation, alignment);
size += calculator.calculate_serialized_size(value.rotation, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Transform& value) {
cdr << value.translation;
cdr << value.rotation;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Transform& value) {
cdr >> value.translation;
cdr >> value.rotation;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA_CODEC
#define DIMOS_MESSAGE_5794DD17B1F3005DFC4A8593C55A3E3551A27B373DBE184CAE2B09ADDA45BACA_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::TransformStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.child_frame_id, alignment);
size += calculator.calculate_serialized_size(value.transform, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::TransformStamped& value) {
cdr << value.header;
cdr << value.child_frame_id;
cdr << value.transform;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::TransformStamped& value) {
cdr >> value.header;
cdr >> value.child_frame_id;
cdr >> value.transform;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85_CODEC
#define DIMOS_MESSAGE_6BD6A48F194E447088FF7ACA65612826E1824010D805F6AE25DF73AA728CAA85_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Twist& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.linear, alignment);
size += calculator.calculate_serialized_size(value.angular, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Twist& value) {
cdr << value.linear;
cdr << value.angular;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Twist& value) {
cdr >> value.linear;
cdr >> value.angular;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C_CODEC
#define DIMOS_MESSAGE_0A5E54CDB34F7762DE8FDFD590DD1E6174974574A619814EAF22ADB295D1D47C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::TwistStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.twist, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::TwistStamped& value) {
cdr << value.header;
cdr << value.twist;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::TwistStamped& value) {
cdr >> value.header;
cdr >> value.twist;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD_CODEC
#define DIMOS_MESSAGE_232AA384B8843F34E7A4F1BB7FCE2BBAC9A25DE8787E03939DD6B68304ACE9FD_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::TwistWithCovariance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.twist, alignment);
size += calculator.calculate_serialized_size(value.covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::TwistWithCovariance& value) {
cdr << value.twist;
cdr << value.covariance;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::TwistWithCovariance& value) {
cdr >> value.twist;
cdr >> value.covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6_CODEC
#define DIMOS_MESSAGE_5F4A55BD7B686BD9779320BACA30D7E91C0F3DFF9D5992D0386C5551FB2D19A6_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::TwistWithCovarianceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.twist, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::TwistWithCovarianceStamped& value) {
cdr << value.header;
cdr << value.twist;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::TwistWithCovarianceStamped& value) {
cdr >> value.header;
cdr >> value.twist;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75_CODEC
#define DIMOS_MESSAGE_A60C8783E8A917A10072D81D77C45042EA616BB0341B355FB7210CD8715E3B75_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Vector3Stamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.vector, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Vector3Stamped& value) {
cdr << value.header;
cdr << value.vector;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Vector3Stamped& value) {
cdr >> value.header;
cdr >> value.vector;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387_CODEC
#define DIMOS_MESSAGE_5EAFF809E8CD263CD4CB253E2B5B26ED14F72F7721BD3EA6B3570756CBEB0387_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::VelocityStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.body_frame_id, alignment);
size += calculator.calculate_serialized_size(value.reference_frame_id, alignment);
size += calculator.calculate_serialized_size(value.velocity, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::VelocityStamped& value) {
cdr << value.header;
cdr << value.body_frame_id;
cdr << value.reference_frame_id;
cdr << value.velocity;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::VelocityStamped& value) {
cdr >> value.header;
cdr >> value.body_frame_id;
cdr >> value.reference_frame_id;
cdr >> value.velocity;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732_CODEC
#define DIMOS_MESSAGE_EDDCCFDB47CE0944EF742AF0557628A60DA2BC09F94025FD5C19F7625B75A732_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::VelocityWithCovarianceStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.body_frame_id, alignment);
size += calculator.calculate_serialized_size(value.reference_frame_id, alignment);
size += calculator.calculate_serialized_size(value.velocity, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::VelocityWithCovarianceStamped& value) {
cdr << value.header;
cdr << value.body_frame_id;
cdr << value.reference_frame_id;
cdr << value.velocity;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::VelocityWithCovarianceStamped& value) {
cdr >> value.header;
cdr >> value.body_frame_id;
cdr >> value.reference_frame_id;
cdr >> value.velocity;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12_CODEC
#define DIMOS_MESSAGE_E68203E57617DD46588647F88AE42282F88379778F2F5AD812FFC400A4D4FC12_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::Wrench& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.force, alignment);
size += calculator.calculate_serialized_size(value.torque, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::Wrench& value) {
cdr << value.force;
cdr << value.torque;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::Wrench& value) {
cdr >> value.force;
cdr >> value.torque;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB_CODEC
#define DIMOS_MESSAGE_3AD3FB2CBF400E0F6788651CE60D90A51B88DA06C0D88F8EEA4DBB7A3CCD00FB_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const geometry_msgs::msg::WrenchStamped& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.wrench, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const geometry_msgs::msg::WrenchStamped& value) {
cdr << value.header;
cdr << value.wrench;
}
template<> inline void deserialize(Cdr& cdr, geometry_msgs::msg::WrenchStamped& value) {
cdr >> value.header;
cdr >> value.wrench;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0_CODEC
#define DIMOS_MESSAGE_54AB3B2F7425273A2A2D95CFEB87E6665E5167144DA9D587658AA3CA28A31EF0_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::Goals& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.goals, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::Goals& value) {
cdr << value.header;
cdr << value.goals;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::Goals& value) {
cdr >> value.header;
cdr >> value.goals;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B_CODEC
#define DIMOS_MESSAGE_52E934DAA814BAFD1E46A0159CA5093B3A6750D2C34648DE2FD886945ADADB3B_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::GridCells& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.cell_width, alignment);
size += calculator.calculate_serialized_size(value.cell_height, alignment);
size += calculator.calculate_serialized_size(value.cells, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::GridCells& value) {
cdr << value.header;
cdr << value.cell_width;
cdr << value.cell_height;
cdr << value.cells;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::GridCells& value) {
cdr >> value.header;
cdr >> value.cell_width;
cdr >> value.cell_height;
cdr >> value.cells;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0_CODEC
#define DIMOS_MESSAGE_7E04A9938BFD27300BAC30FDD7CBBEDD1A91A1646187A746FC6D33C570A48FA0_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::MapMetaData& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.map_load_time, alignment);
size += calculator.calculate_serialized_size(value.resolution, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.origin, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::MapMetaData& value) {
cdr << value.map_load_time;
cdr << value.resolution;
cdr << value.width;
cdr << value.height;
cdr << value.origin;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::MapMetaData& value) {
cdr >> value.map_load_time;
cdr >> value.resolution;
cdr >> value.width;
cdr >> value.height;
cdr >> value.origin;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D_CODEC
#define DIMOS_MESSAGE_562AED13557E94292B295353DFBFFC8B60280E2D85DEF300B11FA7439DC7178D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::OccupancyGrid& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.info, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::OccupancyGrid& value) {
cdr << value.header;
cdr << value.info;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::OccupancyGrid& value) {
cdr >> value.header;
cdr >> value.info;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24_CODEC
#define DIMOS_MESSAGE_A22BB29B4029D8FB78072851860F9D2010925D7FFF23A3B61C96320EDE645F24_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::Odometry& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.child_frame_id, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.twist, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::Odometry& value) {
cdr << value.header;
cdr << value.child_frame_id;
cdr << value.pose;
cdr << value.twist;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::Odometry& value) {
cdr >> value.header;
cdr >> value.child_frame_id;
cdr >> value.pose;
cdr >> value.twist;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7_CODEC
#define DIMOS_MESSAGE_E1135EBB382D9827643E271B5B6D907736A021735728F626E7716C03A98141A7_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::Path& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.poses, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::Path& value) {
cdr << value.header;
cdr << value.poses;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::Path& value) {
cdr >> value.header;
cdr >> value.poses;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96_CODEC
#define DIMOS_MESSAGE_F37E8F04860EC0097D15F4D2AB19D4940E00CB2D0BC99A94DECF12AC69CDBD96_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::TrajectoryPoint& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.velocity, alignment);
size += calculator.calculate_serialized_size(value.acceleration, alignment);
size += calculator.calculate_serialized_size(value.effort, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::TrajectoryPoint& value) {
cdr << value.header;
cdr << value.pose;
cdr << value.velocity;
cdr << value.acceleration;
cdr << value.effort;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::TrajectoryPoint& value) {
cdr >> value.header;
cdr >> value.pose;
cdr >> value.velocity;
cdr >> value.acceleration;
cdr >> value.effort;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA_CODEC
#define DIMOS_MESSAGE_AAA0EF9DDD2AF1488B93C694101867FA31D4FE46D9BD6690C2C6EEE9B4F5DDDA_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const nav_msgs::msg::Trajectory& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const nav_msgs::msg::Trajectory& value) {
cdr << value.header;
cdr << value.points;
}
template<> inline void deserialize(Cdr& cdr, nav_msgs::msg::Trajectory& value) {
cdr >> value.header;
cdr >> value.points;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5_CODEC
#define DIMOS_MESSAGE_821DDE1FC1843E799CA4519CFC36222EEC718DE7167ED41F43DF17B215BDDAA5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::BatteryState& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.voltage, alignment);
size += calculator.calculate_serialized_size(value.temperature, alignment);
size += calculator.calculate_serialized_size(value.current, alignment);
size += calculator.calculate_serialized_size(value.charge, alignment);
size += calculator.calculate_serialized_size(value.capacity, alignment);
size += calculator.calculate_serialized_size(value.design_capacity, alignment);
size += calculator.calculate_serialized_size(value.percentage, alignment);
size += calculator.calculate_serialized_size(value.power_supply_status, alignment);
size += calculator.calculate_serialized_size(value.power_supply_health, alignment);
size += calculator.calculate_serialized_size(value.power_supply_technology, alignment);
size += calculator.calculate_serialized_size(value.present, alignment);
size += calculator.calculate_serialized_size(value.cell_voltage, alignment);
size += calculator.calculate_serialized_size(value.cell_temperature, alignment);
size += calculator.calculate_serialized_size(value.location, alignment);
size += calculator.calculate_serialized_size(value.serial_number, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::BatteryState& value) {
cdr << value.header;
cdr << value.voltage;
cdr << value.temperature;
cdr << value.current;
cdr << value.charge;
cdr << value.capacity;
cdr << value.design_capacity;
cdr << value.percentage;
cdr << value.power_supply_status;
cdr << value.power_supply_health;
cdr << value.power_supply_technology;
cdr << value.present;
cdr << value.cell_voltage;
cdr << value.cell_temperature;
cdr << value.location;
cdr << value.serial_number;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::BatteryState& value) {
cdr >> value.header;
cdr >> value.voltage;
cdr >> value.temperature;
cdr >> value.current;
cdr >> value.charge;
cdr >> value.capacity;
cdr >> value.design_capacity;
cdr >> value.percentage;
cdr >> value.power_supply_status;
cdr >> value.power_supply_health;
cdr >> value.power_supply_technology;
cdr >> value.present;
cdr >> value.cell_voltage;
cdr >> value.cell_temperature;
cdr >> value.location;
cdr >> value.serial_number;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62_CODEC
#define DIMOS_MESSAGE_797F0657E0F0729E2AC8362776D36BB965A8FAD5C7E356369B182484D6D35E62_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::RegionOfInterest& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.x_offset, alignment);
size += calculator.calculate_serialized_size(value.y_offset, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.do_rectify, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::RegionOfInterest& value) {
cdr << value.x_offset;
cdr << value.y_offset;
cdr << value.height;
cdr << value.width;
cdr << value.do_rectify;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::RegionOfInterest& value) {
cdr >> value.x_offset;
cdr >> value.y_offset;
cdr >> value.height;
cdr >> value.width;
cdr >> value.do_rectify;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5_CODEC
#define DIMOS_MESSAGE_ED0D11047B735A3E2BA14A282109A6D2AEFA01164D2771AFC3B39119668FC3C5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::CameraInfo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.distortion_model, alignment);
size += calculator.calculate_serialized_size(value.d, alignment);
size += calculator.calculate_serialized_size(value.k, alignment);
size += calculator.calculate_serialized_size(value.r, alignment);
size += calculator.calculate_serialized_size(value.p, alignment);
size += calculator.calculate_serialized_size(value.binning_x, alignment);
size += calculator.calculate_serialized_size(value.binning_y, alignment);
size += calculator.calculate_serialized_size(value.roi, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::CameraInfo& value) {
cdr << value.header;
cdr << value.height;
cdr << value.width;
cdr << value.distortion_model;
cdr << value.d;
cdr << value.k;
cdr << value.r;
cdr << value.p;
cdr << value.binning_x;
cdr << value.binning_y;
cdr << value.roi;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::CameraInfo& value) {
cdr >> value.header;
cdr >> value.height;
cdr >> value.width;
cdr >> value.distortion_model;
cdr >> value.d;
cdr >> value.k;
cdr >> value.r;
cdr >> value.p;
cdr >> value.binning_x;
cdr >> value.binning_y;
cdr >> value.roi;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6_CODEC
#define DIMOS_MESSAGE_CB3635C584AA4E13BD7960A2AB94AC32964E06569B6ECB83C1ECF21B071C53A6_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::ChannelFloat32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.values, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::ChannelFloat32& value) {
cdr << value.name;
cdr << value.values;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::ChannelFloat32& value) {
cdr >> value.name;
cdr >> value.values;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A_CODEC
#define DIMOS_MESSAGE_E7D8649CF0B305AE1B1A640981E08C762EB9BA89115519175082DE2AB4B2F10A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::CompressedImage& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.format, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::CompressedImage& value) {
cdr << value.header;
cdr << value.format;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::CompressedImage& value) {
cdr >> value.header;
cdr >> value.format;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87_CODEC
#define DIMOS_MESSAGE_B8D86BD400C73CA22D5C1D71FDE476270737EF6C95DDF8A44225FCB1B1ED9D87_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::FluidPressure& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.fluid_pressure, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::FluidPressure& value) {
cdr << value.header;
cdr << value.fluid_pressure;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::FluidPressure& value) {
cdr >> value.header;
cdr >> value.fluid_pressure;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733_CODEC
#define DIMOS_MESSAGE_401DCC0E7A563C78F55C1AC2276B9C666B2578BD1A785216167E84F31384D733_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Illuminance& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.illuminance, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Illuminance& value) {
cdr << value.header;
cdr << value.illuminance;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Illuminance& value) {
cdr >> value.header;
cdr >> value.illuminance;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25_CODEC
#define DIMOS_MESSAGE_ACCEE452C3E8A40600752A3C541CF8C4703D0190EFB87F0BF33EBACEA437DD25_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Image& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.encoding, alignment);
size += calculator.calculate_serialized_size(value.is_bigendian, alignment);
size += calculator.calculate_serialized_size(value.step, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Image& value) {
cdr << value.header;
cdr << value.height;
cdr << value.width;
cdr << value.encoding;
cdr << value.is_bigendian;
cdr << value.step;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Image& value) {
cdr >> value.header;
cdr >> value.height;
cdr >> value.width;
cdr >> value.encoding;
cdr >> value.is_bigendian;
cdr >> value.step;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75_CODEC
#define DIMOS_MESSAGE_968BEF362FE7379CCB91A2E6B1F2B1AA1D841BDF7FE71EA0E01E1BA09C382B75_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Imu& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.orientation, alignment);
size += calculator.calculate_serialized_size(value.orientation_covariance, alignment);
size += calculator.calculate_serialized_size(value.angular_velocity, alignment);
size += calculator.calculate_serialized_size(value.angular_velocity_covariance, alignment);
size += calculator.calculate_serialized_size(value.linear_acceleration, alignment);
size += calculator.calculate_serialized_size(value.linear_acceleration_covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Imu& value) {
cdr << value.header;
cdr << value.orientation;
cdr << value.orientation_covariance;
cdr << value.angular_velocity;
cdr << value.angular_velocity_covariance;
cdr << value.linear_acceleration;
cdr << value.linear_acceleration_covariance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Imu& value) {
cdr >> value.header;
cdr >> value.orientation;
cdr >> value.orientation_covariance;
cdr >> value.angular_velocity;
cdr >> value.angular_velocity_covariance;
cdr >> value.linear_acceleration;
cdr >> value.linear_acceleration_covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752_CODEC
#define DIMOS_MESSAGE_A44D2CC94779D4F3BABD4591892982BBC8EBF0170918889ABC8513A158E45752_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::JointState& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.position, alignment);
size += calculator.calculate_serialized_size(value.velocity, alignment);
size += calculator.calculate_serialized_size(value.effort, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::JointState& value) {
cdr << value.header;
cdr << value.name;
cdr << value.position;
cdr << value.velocity;
cdr << value.effort;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::JointState& value) {
cdr >> value.header;
cdr >> value.name;
cdr >> value.position;
cdr >> value.velocity;
cdr >> value.effort;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8_CODEC
#define DIMOS_MESSAGE_80E1CA4BD98C3EACB9244CFF8BA14C80784F82B4BF128463E4B033CD9E8EF0A8_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Joy& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.axes, alignment);
size += calculator.calculate_serialized_size(value.buttons, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Joy& value) {
cdr << value.header;
cdr << value.axes;
cdr << value.buttons;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Joy& value) {
cdr >> value.header;
cdr >> value.axes;
cdr >> value.buttons;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F_CODEC
#define DIMOS_MESSAGE_E831230DD9AECF28122DDA419CF9A6A55803FC39CB16FB5BF017E483A3614C1F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::JoyFeedback& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
size += calculator.calculate_serialized_size(value.intensity, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::JoyFeedback& value) {
cdr << value.type;
cdr << value.id;
cdr << value.intensity;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::JoyFeedback& value) {
cdr >> value.type;
cdr >> value.id;
cdr >> value.intensity;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF_CODEC
#define DIMOS_MESSAGE_6D8586A4B543C3B8B7B7DCA4E0096CE303D6A6277E183C288FD99BA620B6C5FF_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::JoyFeedbackArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.array, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::JoyFeedbackArray& value) {
cdr << value.array;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::JoyFeedbackArray& value) {
cdr >> value.array;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71_CODEC
#define DIMOS_MESSAGE_737A5D9361D971C50782976050BABCB1B3A68540ACF7248494B85EF0F790DE71_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::LaserEcho& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.echoes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::LaserEcho& value) {
cdr << value.echoes;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::LaserEcho& value) {
cdr >> value.echoes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419_CODEC
#define DIMOS_MESSAGE_6D8A5A5CD444784FE66B80335DA273795991350B02DE0D9CC8862B8FBD7A8419_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::LaserScan& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.angle_min, alignment);
size += calculator.calculate_serialized_size(value.angle_max, alignment);
size += calculator.calculate_serialized_size(value.angle_increment, alignment);
size += calculator.calculate_serialized_size(value.time_increment, alignment);
size += calculator.calculate_serialized_size(value.scan_time, alignment);
size += calculator.calculate_serialized_size(value.range_min, alignment);
size += calculator.calculate_serialized_size(value.range_max, alignment);
size += calculator.calculate_serialized_size(value.ranges, alignment);
size += calculator.calculate_serialized_size(value.intensities, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::LaserScan& value) {
cdr << value.header;
cdr << value.angle_min;
cdr << value.angle_max;
cdr << value.angle_increment;
cdr << value.time_increment;
cdr << value.scan_time;
cdr << value.range_min;
cdr << value.range_max;
cdr << value.ranges;
cdr << value.intensities;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::LaserScan& value) {
cdr >> value.header;
cdr >> value.angle_min;
cdr >> value.angle_max;
cdr >> value.angle_increment;
cdr >> value.time_increment;
cdr >> value.scan_time;
cdr >> value.range_min;
cdr >> value.range_max;
cdr >> value.ranges;
cdr >> value.intensities;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E_CODEC
#define DIMOS_MESSAGE_D70C60CF1B6FE8568199D45FC82EE562A283A5C7524D113D20C624750B234D0E_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::MagneticField& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.magnetic_field, alignment);
size += calculator.calculate_serialized_size(value.magnetic_field_covariance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::MagneticField& value) {
cdr << value.header;
cdr << value.magnetic_field;
cdr << value.magnetic_field_covariance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::MagneticField& value) {
cdr >> value.header;
cdr >> value.magnetic_field;
cdr >> value.magnetic_field_covariance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C_CODEC
#define DIMOS_MESSAGE_C451B120BE64FE7A80E067332979CDD4011CA15969D956A0CED72D7DF82AF63C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::MultiDOFJointState& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.joint_names, alignment);
size += calculator.calculate_serialized_size(value.transforms, alignment);
size += calculator.calculate_serialized_size(value.twist, alignment);
size += calculator.calculate_serialized_size(value.wrench, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::MultiDOFJointState& value) {
cdr << value.header;
cdr << value.joint_names;
cdr << value.transforms;
cdr << value.twist;
cdr << value.wrench;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::MultiDOFJointState& value) {
cdr >> value.header;
cdr >> value.joint_names;
cdr >> value.transforms;
cdr >> value.twist;
cdr >> value.wrench;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF_CODEC
#define DIMOS_MESSAGE_0E5C6EC168677E967A145697AFDE1A7D309FC15D4F2CE1E2B38932D9351239AF_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::MultiEchoLaserScan& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.angle_min, alignment);
size += calculator.calculate_serialized_size(value.angle_max, alignment);
size += calculator.calculate_serialized_size(value.angle_increment, alignment);
size += calculator.calculate_serialized_size(value.time_increment, alignment);
size += calculator.calculate_serialized_size(value.scan_time, alignment);
size += calculator.calculate_serialized_size(value.range_min, alignment);
size += calculator.calculate_serialized_size(value.range_max, alignment);
size += calculator.calculate_serialized_size(value.ranges, alignment);
size += calculator.calculate_serialized_size(value.intensities, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::MultiEchoLaserScan& value) {
cdr << value.header;
cdr << value.angle_min;
cdr << value.angle_max;
cdr << value.angle_increment;
cdr << value.time_increment;
cdr << value.scan_time;
cdr << value.range_min;
cdr << value.range_max;
cdr << value.ranges;
cdr << value.intensities;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::MultiEchoLaserScan& value) {
cdr >> value.header;
cdr >> value.angle_min;
cdr >> value.angle_max;
cdr >> value.angle_increment;
cdr >> value.time_increment;
cdr >> value.scan_time;
cdr >> value.range_min;
cdr >> value.range_max;
cdr >> value.ranges;
cdr >> value.intensities;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2_CODEC
#define DIMOS_MESSAGE_5EFBF6A91B195B289351D12A543F621E1B4400D745A87556A72E1F395A6A46F2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::NavSatStatus& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.status, alignment);
size += calculator.calculate_serialized_size(value.service, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::NavSatStatus& value) {
cdr << value.status;
cdr << value.service;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::NavSatStatus& value) {
cdr >> value.status;
cdr >> value.service;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9_CODEC
#define DIMOS_MESSAGE_AA3869FEDE86C190E5B37E8695E9E668F88C3F1DB2AECD941317FEB0F28823B9_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::NavSatFix& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.status, alignment);
size += calculator.calculate_serialized_size(value.latitude, alignment);
size += calculator.calculate_serialized_size(value.longitude, alignment);
size += calculator.calculate_serialized_size(value.altitude, alignment);
size += calculator.calculate_serialized_size(value.position_covariance, alignment);
size += calculator.calculate_serialized_size(value.position_covariance_type, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::NavSatFix& value) {
cdr << value.header;
cdr << value.status;
cdr << value.latitude;
cdr << value.longitude;
cdr << value.altitude;
cdr << value.position_covariance;
cdr << value.position_covariance_type;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::NavSatFix& value) {
cdr >> value.header;
cdr >> value.status;
cdr >> value.latitude;
cdr >> value.longitude;
cdr >> value.altitude;
cdr >> value.position_covariance;
cdr >> value.position_covariance_type;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D_CODEC
#define DIMOS_MESSAGE_0093B4F03030D26CC0A0093B53CAD4555F31FDC2900BE6A721C7C15041CB282D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::PointCloud& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
size += calculator.calculate_serialized_size(value.channels, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::PointCloud& value) {
cdr << value.header;
cdr << value.points;
cdr << value.channels;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::PointCloud& value) {
cdr >> value.header;
cdr >> value.points;
cdr >> value.channels;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018_CODEC
#define DIMOS_MESSAGE_5BDC8B8CAC909977ACD1C7C68D796B7A13B1379C42B5F6567F62DFAF1B8B1018_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::PointField& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.offset, alignment);
size += calculator.calculate_serialized_size(value.datatype, alignment);
size += calculator.calculate_serialized_size(value.count, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::PointField& value) {
cdr << value.name;
cdr << value.offset;
cdr << value.datatype;
cdr << value.count;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::PointField& value) {
cdr >> value.name;
cdr >> value.offset;
cdr >> value.datatype;
cdr >> value.count;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973_CODEC
#define DIMOS_MESSAGE_22C2127CA493475C527B387516755C5949FD90D611E932BA3939DF8D918A3973_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::PointCloud2& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.height, alignment);
size += calculator.calculate_serialized_size(value.width, alignment);
size += calculator.calculate_serialized_size(value.fields, alignment);
size += calculator.calculate_serialized_size(value.is_bigendian, alignment);
size += calculator.calculate_serialized_size(value.point_step, alignment);
size += calculator.calculate_serialized_size(value.row_step, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
size += calculator.calculate_serialized_size(value.is_dense, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::PointCloud2& value) {
cdr << value.header;
cdr << value.height;
cdr << value.width;
cdr << value.fields;
cdr << value.is_bigendian;
cdr << value.point_step;
cdr << value.row_step;
cdr << value.data;
cdr << value.is_dense;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::PointCloud2& value) {
cdr >> value.header;
cdr >> value.height;
cdr >> value.width;
cdr >> value.fields;
cdr >> value.is_bigendian;
cdr >> value.point_step;
cdr >> value.row_step;
cdr >> value.data;
cdr >> value.is_dense;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579_CODEC
#define DIMOS_MESSAGE_F6AF5BF7194DD3428574B3CE4084DBF57783541437AEFDFC506A04BD4CC82579_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Range& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.radiation_type, alignment);
size += calculator.calculate_serialized_size(value.field_of_view, alignment);
size += calculator.calculate_serialized_size(value.min_range, alignment);
size += calculator.calculate_serialized_size(value.max_range, alignment);
size += calculator.calculate_serialized_size(value.range, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Range& value) {
cdr << value.header;
cdr << value.radiation_type;
cdr << value.field_of_view;
cdr << value.min_range;
cdr << value.max_range;
cdr << value.range;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Range& value) {
cdr >> value.header;
cdr >> value.radiation_type;
cdr >> value.field_of_view;
cdr >> value.min_range;
cdr >> value.max_range;
cdr >> value.range;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0_CODEC
#define DIMOS_MESSAGE_5B7F9746F1425EF5BB132135866A85E9CE178BD1944C2FD855F38D60CCA77AE0_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::RelativeHumidity& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.relative_humidity, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::RelativeHumidity& value) {
cdr << value.header;
cdr << value.relative_humidity;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::RelativeHumidity& value) {
cdr >> value.header;
cdr >> value.relative_humidity;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D_CODEC
#define DIMOS_MESSAGE_9D92C737B1C82163F545AB36DBC36296F0CDF3E51563970122E9FFE56686512D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::Temperature& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.temperature, alignment);
size += calculator.calculate_serialized_size(value.variance, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::Temperature& value) {
cdr << value.header;
cdr << value.temperature;
cdr << value.variance;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::Temperature& value) {
cdr >> value.header;
cdr >> value.temperature;
cdr >> value.variance;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430_CODEC
#define DIMOS_MESSAGE_7681D29CF14CB790367D77DEAA6B241D2C3CA95B4C5549266F204BC911BAA430_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const sensor_msgs::msg::TimeReference& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.time_ref, alignment);
size += calculator.calculate_serialized_size(value.source, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const sensor_msgs::msg::TimeReference& value) {
cdr << value.header;
cdr << value.time_ref;
cdr << value.source;
}
template<> inline void deserialize(Cdr& cdr, sensor_msgs::msg::TimeReference& value) {
cdr >> value.header;
cdr >> value.time_ref;
cdr >> value.source;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50_CODEC
#define DIMOS_MESSAGE_C949E8DEF53DDBC8584EA61D85F3E46F87BF134878A027CC14B0998D29AC3D50_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const shape_msgs::msg::MeshTriangle& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.vertex_indices, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const shape_msgs::msg::MeshTriangle& value) {
cdr << value.vertex_indices;
}
template<> inline void deserialize(Cdr& cdr, shape_msgs::msg::MeshTriangle& value) {
cdr >> value.vertex_indices;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68_CODEC
#define DIMOS_MESSAGE_F17CB28724F8C4AFBD97A86D656F6A2C846AB339B8431CC52873FB36120F0D68_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const shape_msgs::msg::Mesh& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.triangles, alignment);
size += calculator.calculate_serialized_size(value.vertices, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const shape_msgs::msg::Mesh& value) {
cdr << value.triangles;
cdr << value.vertices;
}
template<> inline void deserialize(Cdr& cdr, shape_msgs::msg::Mesh& value) {
cdr >> value.triangles;
cdr >> value.vertices;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5_CODEC
#define DIMOS_MESSAGE_05850CA07AA1F58AEFC0318F62EE5A45CF22EE8F9F4A421E35A3D76A3CAADAB5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const shape_msgs::msg::Plane& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.coef, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const shape_msgs::msg::Plane& value) {
cdr << value.coef;
}
template<> inline void deserialize(Cdr& cdr, shape_msgs::msg::Plane& value) {
cdr >> value.coef;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5_CODEC
#define DIMOS_MESSAGE_6F43CB02FAE4199A952AABDC44C77C0F281449C5B3EDE329B6BCB4C9F8BC69D5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const shape_msgs::msg::SolidPrimitive& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.dimensions, alignment);
size += calculator.calculate_serialized_size(value.polygon, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const shape_msgs::msg::SolidPrimitive& value) {
cdr << value.type;
cdr << value.dimensions;
cdr << value.polygon;
}
template<> inline void deserialize(Cdr& cdr, shape_msgs::msg::SolidPrimitive& value) {
cdr >> value.type;
cdr >> value.dimensions;
cdr >> value.polygon;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B_CODEC
#define DIMOS_MESSAGE_19673FEB22AD12F173164771E6ADAB77E5424716196AA208406F2E9915BD149B_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Bool& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Bool& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Bool& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141_CODEC
#define DIMOS_MESSAGE_53C23BF6EA4B9023AFC06F5792137A101ACF4E84DA47530F0DEDB7BE5F2C3141_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Byte& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Byte& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Byte& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2_CODEC
#define DIMOS_MESSAGE_9C367C937BBFE0A430006F78048522108E3806469E99217FE766C3B9B53749E2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::MultiArrayDimension& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.label, alignment);
size += calculator.calculate_serialized_size(value.size, alignment);
size += calculator.calculate_serialized_size(value.stride, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::MultiArrayDimension& value) {
cdr << value.label;
cdr << value.size;
cdr << value.stride;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::MultiArrayDimension& value) {
cdr >> value.label;
cdr >> value.size;
cdr >> value.stride;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8_CODEC
#define DIMOS_MESSAGE_0F300E1E0DD1049F7FF7ABDA0A18D9CD58E5F4900F75B7D4CED61F8F768E0AE8_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::MultiArrayLayout& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.dim, alignment);
size += calculator.calculate_serialized_size(value.data_offset, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::MultiArrayLayout& value) {
cdr << value.dim;
cdr << value.data_offset;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::MultiArrayLayout& value) {
cdr >> value.dim;
cdr >> value.data_offset;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39_CODEC
#define DIMOS_MESSAGE_70F654FF2C29BE9F6548D16EBC0736267FF2EB716FDE68F856C950E581DF1F39_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::ByteMultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::ByteMultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::ByteMultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7_CODEC
#define DIMOS_MESSAGE_FE6C3C45BA72E1B47EDA668D07416627022556BE2A4A28CE08F391D189BE57C7_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Char& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Char& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Char& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F_CODEC
#define DIMOS_MESSAGE_5434086685B2D8A251847B08532379016B70097F4F4D891A2D1DCBA8DA3B922F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::ColorRGBA& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.r, alignment);
size += calculator.calculate_serialized_size(value.g, alignment);
size += calculator.calculate_serialized_size(value.b, alignment);
size += calculator.calculate_serialized_size(value.a, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::ColorRGBA& value) {
cdr << value.r;
cdr << value.g;
cdr << value.b;
cdr << value.a;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::ColorRGBA& value) {
cdr >> value.r;
cdr >> value.g;
cdr >> value.b;
cdr >> value.a;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380_CODEC
#define DIMOS_MESSAGE_E8B6EC91741D4A29E548E473A4EA481704D56888931B9121AD873241F7BFC380_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Empty& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(uint8_t{0}, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Empty& value) {
cdr << uint8_t{0};
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Empty& value) {
uint8_t unused; cdr >> unused;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC_CODEC
#define DIMOS_MESSAGE_4C31859EDA1C3762C91D20965770D08B43651C0738B25C36B692584A01B418EC_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Float32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Float32& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Float32& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1_CODEC
#define DIMOS_MESSAGE_54179BDF870C5D7BB91EF56847A3137DEFA9E19E978A47680F70E20E57BFD5F1_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Float32MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Float32MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Float32MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33_CODEC
#define DIMOS_MESSAGE_85E0A663C198207B13049F6A289544222C39C8CF649902AA954B2BC20D0DDE33_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Float64& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Float64& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Float64& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330_CODEC
#define DIMOS_MESSAGE_0EFB438BC676D57C3747B166A7BA0D5D2167E981D52FCAFD385A3109F8B36330_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Float64MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Float64MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Float64MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731_CODEC
#define DIMOS_MESSAGE_BF5EB7C55214BE606537ADC29FEF34F255E91687EB125ADAEF5B846749215731_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int16& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int16& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int16& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119_CODEC
#define DIMOS_MESSAGE_171EE8E989D12B1C35F113AB64DE48A39F4BCB684FE0E19863BEE3D4BC435119_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int16MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int16MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int16MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488_CODEC
#define DIMOS_MESSAGE_871CEF34A10C4340EAE5F47918B7DC46FCC0ECCD6A9D89E2786275E6FD4CC488_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int32& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int32& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969_CODEC
#define DIMOS_MESSAGE_F40E3933CB797A5536F7A18CD5F41DCCA27613563D1363305C0900B3E658C969_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int32MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int32MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int32MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D_CODEC
#define DIMOS_MESSAGE_0BB7B3C6385E76D154FB01B2AC5DA2DF08856CF22F977012CAC63864A248FA2D_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int64& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int64& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int64& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F_CODEC
#define DIMOS_MESSAGE_EC72DECA68E6043F6308CE54CD154FB3B109FB9F772329F24FD9F4DD2E8E5B8F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int64MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int64MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int64MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575_CODEC
#define DIMOS_MESSAGE_CB35216E31C109D7B2EF5AB73141E4DE3F226FB01E707675634ECCE81D28B575_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int8& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int8& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int8& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C_CODEC
#define DIMOS_MESSAGE_C88F8D70F30C428E60239163AB8DCC900EEB58DBCFF1A17588A95EA70339D23C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::Int8MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::Int8MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::Int8MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1_CODEC
#define DIMOS_MESSAGE_D8956D4857104EE92C0EA51BBEDFD127AE3259066268ED606EC0186874E07BA1_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::String& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::String& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::String& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5_CODEC
#define DIMOS_MESSAGE_0CF0602BC9BF503D92B26EFBAA5C72BE4A17E44F002A1B31E0D2C8E9148FC6A5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt16& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt16& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt16& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81_CODEC
#define DIMOS_MESSAGE_6A92923648B0D2150F6C37E671CCD684F706EA62063228A1CA442CFC832DEB81_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt16MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt16MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt16MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F_CODEC
#define DIMOS_MESSAGE_CD82171550A79D4BF40007A401495EC7FA98A5277D7E661DA91669D7EEFC0D0F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt32& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt32& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt32& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD_CODEC
#define DIMOS_MESSAGE_16C1E1E348F019BDB0FC3968FB148C72F0E7CF27524B49DF70AFB86A31587EAD_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt32MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt32MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt32MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7_CODEC
#define DIMOS_MESSAGE_30FDE3247BD1533CF961223D6CC9320F5E835551DD27C0EE41C7DC2C67AB6DF7_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt64& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt64& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt64& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95_CODEC
#define DIMOS_MESSAGE_8CA45B1EA19A9456DB014829C9DC883097A65DE5D3266545D5D41EA8B8306E95_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt64MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt64MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt64MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4_CODEC
#define DIMOS_MESSAGE_68730B2CEB05697F8F6672573AE5AE5F594E0DE9E7795C9600FE373B6591F2E4_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt8& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt8& value) {
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt8& value) {
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636_CODEC
#define DIMOS_MESSAGE_9FB78B0C358E8E5E8A1E31C672E8EEF7E5CBDBB2AA3CDD864C1457768FB28636_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const std_msgs::msg::UInt8MultiArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.layout, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const std_msgs::msg::UInt8MultiArray& value) {
cdr << value.layout;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, std_msgs::msg::UInt8MultiArray& value) {
cdr >> value.layout;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C_CODEC
#define DIMOS_MESSAGE_22AF39D92FF41BE39E6CD4AB1CA30CACE8F6187A7EACA2A5C230D59EF0139B6C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const tf2_msgs::msg::TF2Error& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.error, alignment);
size += calculator.calculate_serialized_size(value.error_string, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const tf2_msgs::msg::TF2Error& value) {
cdr << value.error;
cdr << value.error_string;
}
template<> inline void deserialize(Cdr& cdr, tf2_msgs::msg::TF2Error& value) {
cdr >> value.error;
cdr >> value.error_string;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A_CODEC
#define DIMOS_MESSAGE_E1E0F49EF583F4E9C52A26BBFC7C0790821F9C27689D87F64EC4DDB0DB88438A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const tf2_msgs::msg::TFMessage& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.transforms, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const tf2_msgs::msg::TFMessage& value) {
cdr << value.transforms;
}
template<> inline void deserialize(Cdr& cdr, tf2_msgs::msg::TFMessage& value) {
cdr >> value.transforms;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848_CODEC
#define DIMOS_MESSAGE_9BF5ECEEBF3008E723C5205F0FC0B8A830933A26B215671E9180464B3BC5B848_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const trajectory_msgs::msg::JointTrajectoryPoint& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.positions, alignment);
size += calculator.calculate_serialized_size(value.velocities, alignment);
size += calculator.calculate_serialized_size(value.accelerations, alignment);
size += calculator.calculate_serialized_size(value.effort, alignment);
size += calculator.calculate_serialized_size(value.time_from_start, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const trajectory_msgs::msg::JointTrajectoryPoint& value) {
cdr << value.positions;
cdr << value.velocities;
cdr << value.accelerations;
cdr << value.effort;
cdr << value.time_from_start;
}
template<> inline void deserialize(Cdr& cdr, trajectory_msgs::msg::JointTrajectoryPoint& value) {
cdr >> value.positions;
cdr >> value.velocities;
cdr >> value.accelerations;
cdr >> value.effort;
cdr >> value.time_from_start;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56_CODEC
#define DIMOS_MESSAGE_F2D73553F7F5FF4D1A0F4F6DCE4F7192D48DFE1B602869D1EF02F14F57B12D56_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const trajectory_msgs::msg::JointTrajectory& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.joint_names, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const trajectory_msgs::msg::JointTrajectory& value) {
cdr << value.header;
cdr << value.joint_names;
cdr << value.points;
}
template<> inline void deserialize(Cdr& cdr, trajectory_msgs::msg::JointTrajectory& value) {
cdr >> value.header;
cdr >> value.joint_names;
cdr >> value.points;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA_CODEC
#define DIMOS_MESSAGE_9C614FDEC3ACF8E0816F815CA777D1B59B7DFE99D8B41CBF425210C23966DEEA_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.transforms, alignment);
size += calculator.calculate_serialized_size(value.velocities, alignment);
size += calculator.calculate_serialized_size(value.accelerations, alignment);
size += calculator.calculate_serialized_size(value.time_from_start, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& value) {
cdr << value.transforms;
cdr << value.velocities;
cdr << value.accelerations;
cdr << value.time_from_start;
}
template<> inline void deserialize(Cdr& cdr, trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& value) {
cdr >> value.transforms;
cdr >> value.velocities;
cdr >> value.accelerations;
cdr >> value.time_from_start;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2_CODEC
#define DIMOS_MESSAGE_F92E61128D23F1F3062907F2981DE3C8C51534956FC7F9C327738C1B019DE0B2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const trajectory_msgs::msg::MultiDOFJointTrajectory& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.joint_names, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const trajectory_msgs::msg::MultiDOFJointTrajectory& value) {
cdr << value.header;
cdr << value.joint_names;
cdr << value.points;
}
template<> inline void deserialize(Cdr& cdr, trajectory_msgs::msg::MultiDOFJointTrajectory& value) {
cdr >> value.header;
cdr >> value.joint_names;
cdr >> value.points;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E_CODEC
#define DIMOS_MESSAGE_7FC1C825966ACD0BBAB7FF6ECBD6AF7CCCA38206586D36679E394D835264ED6E_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::BoundingBox2DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.boxes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::BoundingBox2DArray& value) {
cdr << value.header;
cdr << value.boxes;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::BoundingBox2DArray& value) {
cdr >> value.header;
cdr >> value.boxes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65_CODEC
#define DIMOS_MESSAGE_420A25159CB1D5B65FC5AC2E0868CCD8DC37DFE10778D89DE7E048BFC7286C65_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::BoundingBox3DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.boxes, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::BoundingBox3DArray& value) {
cdr << value.header;
cdr << value.boxes;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::BoundingBox3DArray& value) {
cdr >> value.header;
cdr >> value.boxes;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A_CODEC
#define DIMOS_MESSAGE_2BDFB4D9E60F0334B24AED810BA1020AD0988B157AAD4564BDF8C2EC66AAF67A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::ObjectHypothesis& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.class_id, alignment);
size += calculator.calculate_serialized_size(value.score, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::ObjectHypothesis& value) {
cdr << value.class_id;
cdr << value.score;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::ObjectHypothesis& value) {
cdr >> value.class_id;
cdr >> value.score;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339_CODEC
#define DIMOS_MESSAGE_46FAA33FAADD8929594F135C509CDD62DEC7580F4923C91E603D6E5C7624D339_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Classification& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.results, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Classification& value) {
cdr << value.header;
cdr << value.results;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Classification& value) {
cdr >> value.header;
cdr >> value.results;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1_CODEC
#define DIMOS_MESSAGE_D577B871793124E2CEA09966650BB31FE6602FFEBC0AD297F556352FB17A33E1_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::ObjectHypothesisWithPose& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.hypothesis, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::ObjectHypothesisWithPose& value) {
cdr << value.hypothesis;
cdr << value.pose;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::ObjectHypothesisWithPose& value) {
cdr >> value.hypothesis;
cdr >> value.pose;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29_CODEC
#define DIMOS_MESSAGE_8179C9252123FFB0E3E0FA29774DC6BE822BF6648FA865E2E8DB5B17CC6F2B29_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Detection2D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.results, alignment);
size += calculator.calculate_serialized_size(value.bbox, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Detection2D& value) {
cdr << value.header;
cdr << value.results;
cdr << value.bbox;
cdr << value.id;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Detection2D& value) {
cdr >> value.header;
cdr >> value.results;
cdr >> value.bbox;
cdr >> value.id;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A_CODEC
#define DIMOS_MESSAGE_DE3D9F912660340D5007041116B65B79E466054C36DC36F71216AAD0A197A68A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Detection2DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.detections, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Detection2DArray& value) {
cdr << value.header;
cdr << value.detections;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Detection2DArray& value) {
cdr >> value.header;
cdr >> value.detections;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C_CODEC
#define DIMOS_MESSAGE_103AF15F48A00A6EDA2E1A19F6F753367ACA23DF1808B3C7616643D837E05C4C_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Detection3D& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.results, alignment);
size += calculator.calculate_serialized_size(value.bbox, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Detection3D& value) {
cdr << value.header;
cdr << value.results;
cdr << value.bbox;
cdr << value.id;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Detection3D& value) {
cdr >> value.header;
cdr >> value.results;
cdr >> value.bbox;
cdr >> value.id;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE_CODEC
#define DIMOS_MESSAGE_2760AC81EF3C8A5B1D9D22B2DEB38B17E78266F4907D80D02E97CA1D2E7841CE_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::Detection3DArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.detections, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::Detection3DArray& value) {
cdr << value.header;
cdr << value.detections;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::Detection3DArray& value) {
cdr >> value.header;
cdr >> value.detections;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A_CODEC
#define DIMOS_MESSAGE_2352A8302BF3FFB7269B6EC906B5F2E9C6C9CFC3DC7F64D5D7ABB56A520E218A_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::VisionClass& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.class_id, alignment);
size += calculator.calculate_serialized_size(value.class_name, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::VisionClass& value) {
cdr << value.class_id;
cdr << value.class_name;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::VisionClass& value) {
cdr >> value.class_id;
cdr >> value.class_name;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062_CODEC
#define DIMOS_MESSAGE_942ECB09F272EF4F9A7A057400C99171A1F61215239DA1D68855E56DBB696062_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::LabelInfo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.class_map, alignment);
size += calculator.calculate_serialized_size(value.threshold, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::LabelInfo& value) {
cdr << value.header;
cdr << value.class_map;
cdr << value.threshold;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::LabelInfo& value) {
cdr >> value.header;
cdr >> value.class_map;
cdr >> value.threshold;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2_CODEC
#define DIMOS_MESSAGE_E388C1419181AE346924227BADC760F058F3469C524047887A60D5529A9E97D2_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const vision_msgs::msg::VisionInfo& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.method, alignment);
size += calculator.calculate_serialized_size(value.database_location, alignment);
size += calculator.calculate_serialized_size(value.database_version, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const vision_msgs::msg::VisionInfo& value) {
cdr << value.header;
cdr << value.method;
cdr << value.database_location;
cdr << value.database_version;
}
template<> inline void deserialize(Cdr& cdr, vision_msgs::msg::VisionInfo& value) {
cdr >> value.header;
cdr >> value.method;
cdr >> value.database_location;
cdr >> value.database_version;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC_CODEC
#define DIMOS_MESSAGE_65D9C00FB08A4554618F6C98F482A12B9234AE4FF40F775129C1247EF2F333EC_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::ImageMarker& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.ns, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.action, alignment);
size += calculator.calculate_serialized_size(value.position, alignment);
size += calculator.calculate_serialized_size(value.scale, alignment);
size += calculator.calculate_serialized_size(value.outline_color, alignment);
size += calculator.calculate_serialized_size(value.filled, alignment);
size += calculator.calculate_serialized_size(value.fill_color, alignment);
size += calculator.calculate_serialized_size(value.lifetime, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
size += calculator.calculate_serialized_size(value.outline_colors, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::ImageMarker& value) {
cdr << value.header;
cdr << value.ns;
cdr << value.id;
cdr << value.type;
cdr << value.action;
cdr << value.position;
cdr << value.scale;
cdr << value.outline_color;
cdr << value.filled;
cdr << value.fill_color;
cdr << value.lifetime;
cdr << value.points;
cdr << value.outline_colors;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::ImageMarker& value) {
cdr >> value.header;
cdr >> value.ns;
cdr >> value.id;
cdr >> value.type;
cdr >> value.action;
cdr >> value.position;
cdr >> value.scale;
cdr >> value.outline_color;
cdr >> value.filled;
cdr >> value.fill_color;
cdr >> value.lifetime;
cdr >> value.points;
cdr >> value.outline_colors;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264_CODEC
#define DIMOS_MESSAGE_6DD944269E9FD9B7A761AB6B599B26A7F6272D7C6482AB193E34B5A952929264_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::MeshFile& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.filename, alignment);
size += calculator.calculate_serialized_size(value.data, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::MeshFile& value) {
cdr << value.filename;
cdr << value.data;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::MeshFile& value) {
cdr >> value.filename;
cdr >> value.data;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F_CODEC
#define DIMOS_MESSAGE_D7CB07CA6303AB3698D825BF2F91E562DE641D507E940EAD0ECB38471674F30F_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::UVCoordinate& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.u, alignment);
size += calculator.calculate_serialized_size(value.v, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::UVCoordinate& value) {
cdr << value.u;
cdr << value.v;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::UVCoordinate& value) {
cdr >> value.u;
cdr >> value.v;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF_CODEC
#define DIMOS_MESSAGE_8E51664BF701A96E86DF8F6EC12D53AFF4197BF39E196AA3CF5D6DE1C7FD72BF_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::Marker& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.ns, alignment);
size += calculator.calculate_serialized_size(value.id, alignment);
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.action, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.scale, alignment);
size += calculator.calculate_serialized_size(value.color, alignment);
size += calculator.calculate_serialized_size(value.lifetime, alignment);
size += calculator.calculate_serialized_size(value.frame_locked, alignment);
size += calculator.calculate_serialized_size(value.points, alignment);
size += calculator.calculate_serialized_size(value.colors, alignment);
size += calculator.calculate_serialized_size(value.texture_resource, alignment);
size += calculator.calculate_serialized_size(value.texture, alignment);
size += calculator.calculate_serialized_size(value.uv_coordinates, alignment);
size += calculator.calculate_serialized_size(value.text, alignment);
size += calculator.calculate_serialized_size(value.mesh_resource, alignment);
size += calculator.calculate_serialized_size(value.mesh_file, alignment);
size += calculator.calculate_serialized_size(value.mesh_use_embedded_materials, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::Marker& value) {
cdr << value.header;
cdr << value.ns;
cdr << value.id;
cdr << value.type;
cdr << value.action;
cdr << value.pose;
cdr << value.scale;
cdr << value.color;
cdr << value.lifetime;
cdr << value.frame_locked;
cdr << value.points;
cdr << value.colors;
cdr << value.texture_resource;
cdr << value.texture;
cdr << value.uv_coordinates;
cdr << value.text;
cdr << value.mesh_resource;
cdr << value.mesh_file;
cdr << value.mesh_use_embedded_materials;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::Marker& value) {
cdr >> value.header;
cdr >> value.ns;
cdr >> value.id;
cdr >> value.type;
cdr >> value.action;
cdr >> value.pose;
cdr >> value.scale;
cdr >> value.color;
cdr >> value.lifetime;
cdr >> value.frame_locked;
cdr >> value.points;
cdr >> value.colors;
cdr >> value.texture_resource;
cdr >> value.texture;
cdr >> value.uv_coordinates;
cdr >> value.text;
cdr >> value.mesh_resource;
cdr >> value.mesh_file;
cdr >> value.mesh_use_embedded_materials;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95_CODEC
#define DIMOS_MESSAGE_F376B60D2DB3107510BFB8721A2D993C5FC663C628A6CFBE75FA4A47516FAF95_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerControl& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.orientation, alignment);
size += calculator.calculate_serialized_size(value.orientation_mode, alignment);
size += calculator.calculate_serialized_size(value.interaction_mode, alignment);
size += calculator.calculate_serialized_size(value.always_visible, alignment);
size += calculator.calculate_serialized_size(value.markers, alignment);
size += calculator.calculate_serialized_size(value.independent_marker_orientation, alignment);
size += calculator.calculate_serialized_size(value.description, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerControl& value) {
cdr << value.name;
cdr << value.orientation;
cdr << value.orientation_mode;
cdr << value.interaction_mode;
cdr << value.always_visible;
cdr << value.markers;
cdr << value.independent_marker_orientation;
cdr << value.description;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerControl& value) {
cdr >> value.name;
cdr >> value.orientation;
cdr >> value.orientation_mode;
cdr >> value.interaction_mode;
cdr >> value.always_visible;
cdr >> value.markers;
cdr >> value.independent_marker_orientation;
cdr >> value.description;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618_CODEC
#define DIMOS_MESSAGE_47667B4BE016FE9BECD451757E3C274FC66B49BB2C77112B908A31BA1569F618_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::MenuEntry& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.id, alignment);
size += calculator.calculate_serialized_size(value.parent_id, alignment);
size += calculator.calculate_serialized_size(value.title, alignment);
size += calculator.calculate_serialized_size(value.command, alignment);
size += calculator.calculate_serialized_size(value.command_type, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::MenuEntry& value) {
cdr << value.id;
cdr << value.parent_id;
cdr << value.title;
cdr << value.command;
cdr << value.command_type;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::MenuEntry& value) {
cdr >> value.id;
cdr >> value.parent_id;
cdr >> value.title;
cdr >> value.command;
cdr >> value.command_type;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847_CODEC
#define DIMOS_MESSAGE_EFF116DB7049D830432013EBDF4E4ADCC374A6344D65432B1CA5A3834D5E7847_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarker& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.name, alignment);
size += calculator.calculate_serialized_size(value.description, alignment);
size += calculator.calculate_serialized_size(value.scale, alignment);
size += calculator.calculate_serialized_size(value.menu_entries, alignment);
size += calculator.calculate_serialized_size(value.controls, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarker& value) {
cdr << value.header;
cdr << value.pose;
cdr << value.name;
cdr << value.description;
cdr << value.scale;
cdr << value.menu_entries;
cdr << value.controls;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarker& value) {
cdr >> value.header;
cdr >> value.pose;
cdr >> value.name;
cdr >> value.description;
cdr >> value.scale;
cdr >> value.menu_entries;
cdr >> value.controls;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E_CODEC
#define DIMOS_MESSAGE_306CAD4A8A5A71618355D2146D76F1BF0FC64CEB99B408F157BD18318D70685E_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerFeedback& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.client_id, alignment);
size += calculator.calculate_serialized_size(value.marker_name, alignment);
size += calculator.calculate_serialized_size(value.control_name, alignment);
size += calculator.calculate_serialized_size(value.event_type, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.menu_entry_id, alignment);
size += calculator.calculate_serialized_size(value.mouse_point, alignment);
size += calculator.calculate_serialized_size(value.mouse_point_valid, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerFeedback& value) {
cdr << value.header;
cdr << value.client_id;
cdr << value.marker_name;
cdr << value.control_name;
cdr << value.event_type;
cdr << value.pose;
cdr << value.menu_entry_id;
cdr << value.mouse_point;
cdr << value.mouse_point_valid;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerFeedback& value) {
cdr >> value.header;
cdr >> value.client_id;
cdr >> value.marker_name;
cdr >> value.control_name;
cdr >> value.event_type;
cdr >> value.pose;
cdr >> value.menu_entry_id;
cdr >> value.mouse_point;
cdr >> value.mouse_point_valid;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4_CODEC
#define DIMOS_MESSAGE_CE3DA834855C00EB467D18A3BB0F282B68D1EFC262D6CA34F8F286604C4597C4_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerInit& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.server_id, alignment);
size += calculator.calculate_serialized_size(value.seq_num, alignment);
size += calculator.calculate_serialized_size(value.markers, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerInit& value) {
cdr << value.server_id;
cdr << value.seq_num;
cdr << value.markers;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerInit& value) {
cdr >> value.server_id;
cdr >> value.seq_num;
cdr >> value.markers;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5_CODEC
#define DIMOS_MESSAGE_2534FA1A3164CC21E9FEF65C12AE7F265060001F56F8AE50DEFE8B3F64F81BA5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerPose& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.header, alignment);
size += calculator.calculate_serialized_size(value.pose, alignment);
size += calculator.calculate_serialized_size(value.name, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerPose& value) {
cdr << value.header;
cdr << value.pose;
cdr << value.name;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerPose& value) {
cdr >> value.header;
cdr >> value.pose;
cdr >> value.name;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5_CODEC
#define DIMOS_MESSAGE_6953D8BCA7F93E1437AC5B62D90EDE0316875EF6FA4B8D3ED3998725E3365AD5_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::InteractiveMarkerUpdate& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.server_id, alignment);
size += calculator.calculate_serialized_size(value.seq_num, alignment);
size += calculator.calculate_serialized_size(value.type, alignment);
size += calculator.calculate_serialized_size(value.markers, alignment);
size += calculator.calculate_serialized_size(value.poses, alignment);
size += calculator.calculate_serialized_size(value.erases, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::InteractiveMarkerUpdate& value) {
cdr << value.server_id;
cdr << value.seq_num;
cdr << value.type;
cdr << value.markers;
cdr << value.poses;
cdr << value.erases;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::InteractiveMarkerUpdate& value) {
cdr >> value.server_id;
cdr >> value.seq_num;
cdr >> value.type;
cdr >> value.markers;
cdr >> value.poses;
cdr >> value.erases;
value.validate();
}
#endif
#ifndef DIMOS_MESSAGE_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22_CODEC
#define DIMOS_MESSAGE_0E38350AE05224D45663FDA59B543A4B29E962EED8569C35CBF135B5A8F6AC22_CODEC
template<> inline size_t calculate_serialized_size(CdrSizeCalculator& calculator, const visualization_msgs::msg::MarkerArray& value, size_t& alignment) {
size_t size = 0;
size += calculator.calculate_serialized_size(value.markers, alignment);
return size;
}
template<> inline void serialize(Cdr& cdr, const visualization_msgs::msg::MarkerArray& value) {
cdr << value.markers;
}
template<> inline void deserialize(Cdr& cdr, visualization_msgs::msg::MarkerArray& value) {
cdr >> value.markers;
value.validate();
}
#endif
}
