import { red, yellow, green, cyan, bold, dim } from "https://deno.land/std@0.224.0/fmt/colors.ts"
import { Select } from "https://deno.land/x/cliffy@v1.0.0-rc.4/prompt/mod.ts"
import { Checkbox } from "https://deno.land/x/cliffy@v1.0.0-rc.4/prompt/mod.ts";

export function clearScreen() {
    console.log('\x1B[2J')
}

//
// coloring/styling
//
export function header(text) {
    console.log(green(bold(text)))
}

export function subHeader(text) {
    console.log(yellow(bold(text)))
}

export function boringLog(text) {
    console.log(dim(text))
}

export function error(text) {
    console.log(red(text))
}

export function warning(text) {
    console.log(yellow(text))
}

export function highlight(text) {
    return cyan(text)
}


//
// prompts
//
const builtinConfirm = globalThis.confirm
const builtinPrompt = globalThis.prompt

export function confirm(text) {
    // make fancy later
    return builtinConfirm(text)
}

export function prompt(text) {
    // make fancy later
    return builtinPrompt(text)
}

export function askYesNo(question) {
    while (true) {
        let answer = builtinPrompt(question)
        const match = `${answer}`.match(/^ *(y|yes|n|no) *\n?$/i)
        if (match) {
            // if yes
            if (match[1][0] == 'y' || match[1][0] == 'Y') {
                return true
            } else {
                return false
            }
        } else {
            console.log("[ please respond with y/n, yes/no, or use CTRL+C to cancel ]")
        }
    }
}

export async function pickOne(message, {options}) {
    let obj = {}
    if (options instanceof Array) {
        for (let eachString of options) {
            obj[eachString] = eachString
        }
    } else {
        obj = options
    }
    return await Select.prompt({
        message,
        options: Object.entries(obj).map(([key, value]) => ({ name: value, value: key })),
    })
}

export async function pickMany(message, {options}) {
    let obj = {}
    if (options instanceof Array) {
        for (let eachString of options) {
            obj[eachString] = eachString
        }
    } else {
        obj = options
    }
    const selections = await Checkbox.prompt({
        message,
        options: Object.entries(obj).map(([key, value]) => ({ name: value, value: key })),
    })
}
