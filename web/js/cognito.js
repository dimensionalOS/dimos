// Cognito client — talks directly to the cognito-idp public API (no SDK).
// The broker only verifies tokens; all sign-in/sign-up flows live here.
// Pool/client IDs come from the broker's public /auth/config endpoint.

import { brokerOrigin } from './api.js';

let cfg = null;

async function config() {
    if (!cfg) {
        const res = await fetch(`${brokerOrigin()}/api/v1/auth/config`);
        if (!res.ok) throw new Error('Auth service unavailable');
        cfg = await res.json();
    }
    return cfg;
}

async function cognito(target, body) {
    const { region } = await config();
    const res = await fetch(`https://cognito-idp.${region}.amazonaws.com/`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/x-amz-json-1.1',
            'X-Amz-Target': `AWSCognitoIdentityProviderService.${target}`,
        },
        body: JSON.stringify(body),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
        const err = new Error(friendlyError(data));
        err.code = data.__type || '';
        throw err;
    }
    return data;
}

function friendlyError(data) {
    const type = (data.__type || '').split('#').pop();
    const msg = data.message || data.Message || '';
    switch (type) {
        case 'NotAuthorizedException': return msg.includes('expired') ? 'Session expired — log in again' : 'Incorrect email or password';
        case 'UsernameExistsException': return 'An account with this email already exists';
        case 'CodeMismatchException': return 'Incorrect verification code';
        case 'ExpiredCodeException': return 'Verification code expired — request a new one';
        case 'InvalidPasswordException': return 'Password must be at least 8 characters';
        case 'LimitExceededException': return 'Too many attempts — wait a bit and try again';
        case 'UserNotConfirmedException': return 'UserNotConfirmedException';
        default: return msg || type || 'Authentication failed';
    }
}

export async function signUp(email, password) {
    const { client_id } = await config();
    return cognito('SignUp', {
        ClientId: client_id,
        Username: email,
        Password: password,
        UserAttributes: [{ Name: 'email', Value: email }],
    });
}

export async function confirmSignUp(email, code) {
    const { client_id } = await config();
    return cognito('ConfirmSignUp', { ClientId: client_id, Username: email, ConfirmationCode: code });
}

export async function resendCode(email) {
    const { client_id } = await config();
    return cognito('ResendConfirmationCode', { ClientId: client_id, Username: email });
}

export async function login(email, password) {
    const { client_id } = await config();
    const data = await cognito('InitiateAuth', {
        ClientId: client_id,
        AuthFlow: 'USER_PASSWORD_AUTH',
        AuthParameters: { USERNAME: email, PASSWORD: password },
    });
    return data.AuthenticationResult;  // { IdToken, RefreshToken, ExpiresIn, ... }
}

export async function refreshTokens(refreshToken) {
    const { client_id } = await config();
    const data = await cognito('InitiateAuth', {
        ClientId: client_id,
        AuthFlow: 'REFRESH_TOKEN_AUTH',
        AuthParameters: { REFRESH_TOKEN: refreshToken },
    });
    return data.AuthenticationResult;  // { IdToken, ExpiresIn, ... } (no new RefreshToken)
}

export async function forgotPassword(email) {
    const { client_id } = await config();
    return cognito('ForgotPassword', { ClientId: client_id, Username: email });
}

export async function confirmForgotPassword(email, code, password) {
    const { client_id } = await config();
    return cognito('ConfirmForgotPassword', {
        ClientId: client_id, Username: email, ConfirmationCode: code, Password: password,
    });
}

// Decode a JWT payload without verifying (the broker verifies; this is just
// for reading exp/email client-side).
export function tokenPayload(token) {
    try {
        return JSON.parse(atob(token.split('.')[1].replace(/-/g, '+').replace(/_/g, '/')));
    } catch {
        return null;
    }
}

export function tokenExpired(token, skewSeconds = 60) {
    const payload = tokenPayload(token);
    return !payload || (payload.exp * 1000) < Date.now() + skewSeconds * 1000;
}
