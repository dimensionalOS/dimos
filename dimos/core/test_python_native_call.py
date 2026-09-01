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

import pytest
from pytest_mock import MockerFixture

from dimos.core.python_native_call import main


def test_main_imports_and_invokes_declared_callable(mocker: MockerFixture) -> None:
    function = mocker.Mock()
    module = mocker.Mock(convert=function)
    import_module = mocker.patch("dimos.core.python_native_call.import_module", return_value=module)

    main(["package.module:convert", "input.mcap", "dataset"])

    import_module.assert_called_once_with("package.module")
    function.assert_called_once_with("input.mcap", "dataset")


def test_main_requires_a_callable_declaration() -> None:
    with pytest.raises(SystemExit, match="module:function"):
        main([])
