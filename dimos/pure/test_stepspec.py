#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""T3 test plan stubs — names per tasks/t3-validation.md §12.

Modules under test are defined inside test functions with nested bundles and
classify() is called directly: no T2, no engine imports. Cross-module and
eager-annotation cases use exec-created fixture modules (spec §12 preamble).
"""

import pytest

pytestmark = pytest.mark.skip(
    reason="T3 skeleton — implementation pending (tasks/t3-validation.md)"
)


# ── §12.1 classification happy paths ─────────────────────────────────────────


def test_tagger_floor_zero_ceremony():
    raise NotImplementedError


def test_classify_stateless_multi_out():
    raise NotImplementedError


def test_classify_mealy():
    raise NotImplementedError


def test_classify_mealy_no_skip():
    raise NotImplementedError


def test_classify_shape_abstract():
    raise NotImplementedError


def test_classify_shape_impl_inherits():
    raise NotImplementedError


def test_classify_async_stateless():
    raise NotImplementedError


def test_classify_fold_cross_module():
    raise NotImplementedError


def test_classify_fold_plain_function():
    raise NotImplementedError


def test_config_only_subclass_inherits_owner():
    raise NotImplementedError


def test_classify_optional_spellings():
    raise NotImplementedError


def test_classify_typing_spellings():
    raise NotImplementedError


def test_classify_wrapped_step():
    raise NotImplementedError


def test_classify_generator_fold_with_generator_return():
    raise NotImplementedError


# ── §12.2 resolution matrix ──────────────────────────────────────────────────


def test_resolve_bare_nested_stringized():
    raise NotImplementedError


def test_resolve_bare_nested_eager():
    raise NotImplementedError


def test_resolve_self_qualified_in_own_body():
    raise NotImplementedError


def test_resolve_shape_qualified_in_subclass():
    raise NotImplementedError


def test_resolve_cross_module_bundle():
    raise NotImplementedError


def test_resolve_explicit_string_annotations():
    raise NotImplementedError


def test_resolve_redeclared_bare_in_subclass():
    raise NotImplementedError


def test_resolve_class_defined_in_function():
    raise NotImplementedError


def test_resolve_bare_in_subclass_fails():
    raise NotImplementedError


def test_resolve_function_local_bundle_fails():
    raise NotImplementedError


def test_resolve_type_checking_import_fails():
    raise NotImplementedError


def test_resolve_attribute_typo():
    raise NotImplementedError


# ── §12.3 error catalog — one per slug (variants included) ───────────────────


def test_err_step_missing():
    raise NotImplementedError


def test_err_step_and_fold():
    raise NotImplementedError


def test_err_step_and_fold_helper():
    raise NotImplementedError


def test_err_step_not_function_static():
    raise NotImplementedError


def test_err_step_not_function_classmethod():
    raise NotImplementedError


def test_err_step_not_function_attr():
    raise NotImplementedError


def test_err_generator_step():
    raise NotImplementedError


def test_err_async_generator_step():
    raise NotImplementedError


def test_err_step_no_self():
    raise NotImplementedError


def test_err_step_params_kwonly():
    raise NotImplementedError


def test_err_step_params_varargs():
    raise NotImplementedError


def test_err_step_params_posonly():
    raise NotImplementedError


def test_err_step_param_default():
    raise NotImplementedError


def test_err_step_arity_zero():
    raise NotImplementedError


def test_err_step_arity_zero_forgot_self():
    raise NotImplementedError


def test_err_step_arity_many():
    raise NotImplementedError


def test_err_async_mealy():
    raise NotImplementedError


def test_err_fold_arity():
    raise NotImplementedError


def test_err_fold_async():
    raise NotImplementedError


def test_err_fold_async_generator():
    raise NotImplementedError


def test_err_step_unannotated_param():
    raise NotImplementedError


def test_err_step_unannotated_return():
    raise NotImplementedError


def test_err_step_unresolvable_bare_in():
    raise NotImplementedError


def test_err_in_not_row():
    raise NotImplementedError


def test_err_in_not_row_union():
    raise NotImplementedError


def test_err_in_not_row_typevar():
    raise NotImplementedError


def test_err_out_not_row():
    raise NotImplementedError


def test_err_out_union():
    raise NotImplementedError


def test_err_step_returns_nothing():
    raise NotImplementedError


def test_err_mealy_returns_nothing():
    raise NotImplementedError


def test_err_step_returns_awaitable_sync():
    raise NotImplementedError


def test_err_step_returns_awaitable_async():
    raise NotImplementedError


def test_err_mealy_no_state():
    raise NotImplementedError


def test_err_state_mismatch():
    raise NotImplementedError


def test_err_state_mismatch_swapped():
    raise NotImplementedError


def test_err_state_mismatch_shadowed():
    raise NotImplementedError


def test_err_state_is_row():
    raise NotImplementedError


def test_err_state_not_default_constructible():
    raise NotImplementedError


def test_err_mealy_return_shape():
    raise NotImplementedError


def test_err_mealy_return_first_slot():
    raise NotImplementedError


def test_err_fold_rows_param():
    raise NotImplementedError


def test_err_fold_return():
    raise NotImplementedError


def test_err_fold_return_iterable():
    raise NotImplementedError


def test_err_state_unused():
    raise NotImplementedError


def test_err_fold_state():
    raise NotImplementedError


def test_err_bundle_shadowed():
    raise NotImplementedError


def test_err_bundle_shadowed_inherited():
    raise NotImplementedError


# ── §12.4 seam and accessor ──────────────────────────────────────────────────


def test_seam_recipe():
    raise NotImplementedError


def test_step_spec_accessor():
    raise NotImplementedError


def test_step_spec_accessor_unclassified():
    raise NotImplementedError


# ── §12.5 purity guarantees ──────────────────────────────────────────────────


def test_classify_never_instantiates():
    raise NotImplementedError


def test_classify_no_mutation():
    raise NotImplementedError
