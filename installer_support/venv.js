import * as dax from "https://esm.sh/@jsr/david__dax@0.43.2/mod.ts" // see: https://github.com/dsherret/dax
import { env, aliases, $stdout, $stderr, initHelpers, iterateOver } from "https://esm.sh/gh/jeff-hykin/bash2deno@0.1.0.2/helpers.js"
import { regex } from 'https://esm.sh/gh/jeff-hykin/good-js@1.18.2.0/source/flattened/regex.js'
let { $, appendTo, overwrite, hasCommand, makeScope, settings, exitCodeOfLastChildProcess } = initHelpers({ dax })

function removeFromPath(pathToRemove) {
    const pattern = regex`(^${pathToRemove}$|^${pathToRemove}:|:${pathToRemove}$|:${pathToRemove}(?=:))`
    env.PATH = env.PATH.replace(pattern, "")
}

/**
 * this deactivates an externally-activated virtual environment
 * @internal
 */
export function _deactivateExternal() {
    // reset PATH
    if (env._OLD_VIRTUAL_PATH.length) {
        // extract old venv bin from the path (better than just full replacement of path)
        if (env.VIRTUAL_ENV.length) {
            removeFromPath(`${env.VIRTUAL_ENV}/bin`)
        } else {
            env.PATH = env._OLD_VIRTUAL_PATH
        }
        delete env._OLD_VIRTUAL_PATH
    }

    // restore PYTHONHOME
    if (env._OLD_VIRTUAL_PYTHONHOME.length) {
        env.PYTHONHOME = env._OLD_VIRTUAL_PYTHONHOME
        delete env._OLD_VIRTUAL_PYTHONHOME
    }

    // restore PS1
    if (env._OLD_VIRTUAL_PS1.length) {
        env.PS1 = env._OLD_VIRTUAL_PS1
        delete env._OLD_VIRTUAL_PS1
    }

    // remove vars
    delete env.VIRTUAL_ENV
    delete env.VIRTUAL_ENV_PROMPT
}


/**
 * Emulates Python venv's `bin/activate` returns a deactivate function
 *
 * @param {string} projectDirectory - replaces "/Users/jeffhykin/repos/dimos/venv"
 * @param {object} [options]
 * @param {boolean} [options.apply=false] - if true, also writes to Deno.env in this process
 * @returns {Function}
 */
export function activateVenv(projectDirectory) {
    const oldEnv = Deno.env.toObject()
    // if already in a virtual environment, deactivate it
    _deactivateExternal()

    env.VIRTUAL_ENV = projectDirectory
    env.PATH = `${env.VIRTUAL_ENV}/bin:${env.PATH}`
    if (env.PYTHONHOME) {
        env._OLD_VIRTUAL_PYTHONHOME = env.PYTHONHOME
        delete env.PYTHONHOME
    }

    const deactivate = () => {
        // remove vars
        delete env.VIRTUAL_ENV
        delete env._OLD_VIRTUAL_PYTHONHOME
        // restore PATH
        removeFromPath(`${projectDirectory}/bin`)
        // restore PYTHONHOME
        if (oldEnv.PYTHONHOME) {
            env.PYTHONHOME = oldEnv.PYTHONHOME
        }
    }

    return deactivate
}
