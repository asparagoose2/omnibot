import defaultSettings from './default';

const env = process.env.REACT_APP_SETTINGS?.toLowerCase() ?? '';

let settings;

switch (env) {
    default: {
        settings = defaultSettings;
        break;
    }
}

export default settings;